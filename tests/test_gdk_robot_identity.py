from __future__ import annotations

from gsa_taskflow_executor.gdk.robot_identity import run_gdk_robot_identity_snapshot
from gsa_taskflow_executor.gdk.session import GdkSessionManager


class FakeGdk:
    init_called = 0

    class GDKRes:
        kSuccess = 0

    @classmethod
    def reset(cls) -> None:
        cls.init_called = 0

    @classmethod
    def gdk_init(cls) -> int:
        cls.init_called += 1
        return cls.GDKRes.kSuccess

    @staticmethod
    def get_robot_aid() -> str:
        return "G2A0004BC01053"


def fake_import_module(_name: str) -> type[FakeGdk]:
    return FakeGdk


def test_robot_identity_snapshot_reads_gdk_aid_as_robot_serial() -> None:
    FakeGdk.reset()

    result = run_gdk_robot_identity_snapshot(
        timeout_ms=1500,
        import_module=fake_import_module,
    )

    assert result["available"] is True
    assert result["backend"] == "agibot_gdk"
    assert result["action"] == "get_robot_identity"
    assert result["robotAid"] == "G2A0004BC01053"
    assert result["robotSerial"] == "G2A0004BC01053"
    assert result["suggestedRobotSerial"] == "G2A0004BC01053"
    assert result["identitySource"] == "agibot_gdk.get_robot_aid"
    assert FakeGdk.init_called == 1
    assert result["gdk_session"]["purpose"] == "get_robot_identity"


def test_robot_identity_returns_busy_when_session_active() -> None:
    manager = GdkSessionManager(import_module=fake_import_module)
    lease = manager.acquire(blocking=False, initialize=False, purpose="motion")
    assert lease is not None
    try:
        result = run_gdk_robot_identity_snapshot(
            timeout_ms=1500,
            import_module=fake_import_module,
            session_manager=manager,
        )
    finally:
        lease.release()

    assert result["available"] is False
    assert result["busy"] is True
    assert result["activePurpose"] == "motion"


def test_robot_identity_reports_missing_get_robot_aid() -> None:
    class MissingAidGdk:
        pass

    result = run_gdk_robot_identity_snapshot(
        timeout_ms=1500,
        import_module=lambda _name: MissingAidGdk,
    )

    assert result["available"] is False
    assert result["errorStage"] == "get_robot_aid_attr"
