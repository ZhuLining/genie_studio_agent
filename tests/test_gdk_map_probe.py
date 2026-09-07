from __future__ import annotations

from gsa_taskflow_executor.gdk.map_probe import run_gdk_map_probe


class FakePosition:
    x = 1.0
    y = 2.0
    z = 0.0


class FakeOrientation:
    x = 0.0
    y = 0.0
    z = 0.0
    w = 1.0


class FakeOrigin:
    position = FakePosition()
    orientation = FakeOrientation()


class FakeGridMap:
    width = 4
    height = 3
    resolution = 0.05
    origin = FakeOrigin()
    data = list(range(12))


class FakeBytesGridMap(FakeGridMap):
    data = bytes(range(12))


class FakeIndexedData:
    def __init__(self, values: list[int]) -> None:
        self.values = values

    def __len__(self) -> int:
        return len(self.values)

    def __getitem__(self, index: int) -> int:
        return self.values[index]


class FakeIndexedGridMap(FakeGridMap):
    data = FakeIndexedData(list(range(12)))


class FakeMapName:
    def __init__(self, map_id: int, name: str, is_curr_map: bool) -> None:
        self.id = map_id
        self.name = name
        self.is_curr_map = is_curr_map


class FakeMapInfo:
    id = 7
    name = "factory-floor"
    gravity = FakePosition()
    grid_map = FakeGridMap()
    walls = [{"x": 0.0, "y": 1.0}]
    infeasible_areas = []
    guide_pts = [{"id": 1, "x": 1.2, "y": 3.4}]


class FakeBytesMapInfo(FakeMapInfo):
    grid_map = FakeBytesGridMap()


class FakeIndexedMapInfo(FakeMapInfo):
    grid_map = FakeIndexedGridMap()


class FakeMapManager:
    def __init__(self) -> None:
        self.get_map_calls: list[int] = []

    def get_all_map(self) -> list[FakeMapName]:
        return [
            FakeMapName(3, "old", False),
            FakeMapName(7, "factory-floor", True),
        ]

    def get_map(self, map_id: int) -> FakeMapInfo:
        self.get_map_calls.append(map_id)
        return FakeMapInfo()


class FakeBytesMapManager(FakeMapManager):
    def get_map(self, map_id: int) -> FakeBytesMapInfo:
        self.get_map_calls.append(map_id)
        return FakeBytesMapInfo()


class FakeIndexedMapManager(FakeMapManager):
    def get_map(self, map_id: int) -> FakeIndexedMapInfo:
        self.get_map_calls.append(map_id)
        return FakeIndexedMapInfo()


class FakeAgibotGdk:
    manager = FakeMapManager()

    class GDKRes:
        kSuccess = 0

    @classmethod
    def gdk_init(cls) -> int:
        return cls.GDKRes.kSuccess

    @classmethod
    def gdk_release(cls) -> None:
        return None

    @classmethod
    def Map(cls) -> FakeMapManager:  # noqa: N802 - mirrors agibot_gdk API
        return cls.manager


def test_gdk_map_probe_reads_all_maps_then_current_map_detail() -> None:
    FakeAgibotGdk.manager = FakeMapManager()

    result = run_gdk_map_probe(
        import_module=lambda _name: FakeAgibotGdk,
        timeout_ms=5000,
    )

    assert result["available"] is True
    assert result["action"] == "get_high_precision_maps"
    assert result["mapCount"] == 2
    assert result["selectedMapId"] == 7
    assert FakeAgibotGdk.manager.get_map_calls == [7]
    assert result["maps"][1]["name"] == "factory-floor"
    assert result["mapDetail"]["gridMap"]["width"] == 4
    assert result["mapDetail"]["gridMap"]["dataLength"] == 12
    assert result["mapDetail"]["guidePoints"]["count"] == 1


def test_gdk_map_probe_uses_requested_map_id() -> None:
    FakeAgibotGdk.manager = FakeMapManager()

    result = run_gdk_map_probe(
        map_id=3,
        import_module=lambda _name: FakeAgibotGdk,
        timeout_ms=5000,
    )

    assert result["available"] is True
    assert result["requestedMapId"] == 3
    assert result["selectedMapId"] == 3
    assert FakeAgibotGdk.manager.get_map_calls == [3]


def test_gdk_map_probe_summarizes_bytes_grid_data() -> None:
    FakeAgibotGdk.manager = FakeBytesMapManager()

    result = run_gdk_map_probe(
        import_module=lambda _name: FakeAgibotGdk,
        timeout_ms=5000,
    )

    assert result["available"] is True
    assert result["mapDetail"]["gridMap"]["dataLength"] == 12
    assert result["mapDetail"]["gridMap"]["dataSample"] == list(range(12))


def test_gdk_map_probe_summarizes_indexed_grid_data() -> None:
    FakeAgibotGdk.manager = FakeIndexedMapManager()

    result = run_gdk_map_probe(
        import_module=lambda _name: FakeAgibotGdk,
        timeout_ms=5000,
    )

    assert result["available"] is True
    assert result["mapDetail"]["gridMap"]["expectedDataLength"] == 12
    assert result["mapDetail"]["gridMap"]["dataLength"] == 12
    assert result["mapDetail"]["gridMap"]["dataSample"] == list(range(12))
    assert result["mapDetail"]["gridMap"]["dataType"].endswith("FakeIndexedData")


def test_gdk_map_probe_rejects_invalid_map_id_before_importing() -> None:
    def forbidden_import(_name: str) -> object:
        raise AssertionError("invalid map_id must not import GDK")

    result = run_gdk_map_probe(
        map_id=300,
        import_module=forbidden_import,
        timeout_ms=5000,
    )

    assert result["available"] is False
    assert result["errorStage"] == "validate_map_id"
