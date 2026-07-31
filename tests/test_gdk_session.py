from __future__ import annotations

from gsa_taskflow_executor.gdk.session import GdkSessionManager


class FakeAgibotGdk:
    init_called = 0
    release_called = 0

    class GDKRes:
        kSuccess = 0

    @classmethod
    def reset(cls) -> None:
        cls.init_called = 0
        cls.release_called = 0

    @classmethod
    def gdk_init(cls) -> int:
        cls.init_called += 1
        return cls.GDKRes.kSuccess

    @classmethod
    def gdk_release(cls) -> None:
        cls.release_called += 1


def make_manager() -> GdkSessionManager:
    return GdkSessionManager(import_module=lambda _name: FakeAgibotGdk)


def test_session_manager_initializes_gdk_once_across_multiple_leases() -> None:
    FakeAgibotGdk.reset()
    manager = make_manager()

    first = manager.acquire(blocking=True, initialize=True, purpose="motion:first")
    assert first is not None
    with first:
        assert first.init_result["called"] is True
        assert first.init_result["reused"] is False

    second = manager.acquire(blocking=True, initialize=True, purpose="motion:second")
    assert second is not None
    with second:
        assert second.init_result["called"] is True
        assert second.init_result["reused"] is True

    assert FakeAgibotGdk.init_called == 1
    assert FakeAgibotGdk.release_called == 0


def test_session_manager_shutdown_releases_gdk_once_when_idle() -> None:
    FakeAgibotGdk.reset()
    manager = make_manager()

    lease = manager.acquire(blocking=True, initialize=True, purpose="motion")
    assert lease is not None
    with lease:
        pass

    first_shutdown = manager.shutdown()
    second_shutdown = manager.shutdown()

    assert first_shutdown["called"] is True
    assert first_shutdown["success"] is True
    assert second_shutdown["called"] is False
    assert second_shutdown["reason"] == "not_initialized"
    assert FakeAgibotGdk.release_called == 1


def test_session_manager_nonblocking_readonly_acquire_reports_busy() -> None:
    FakeAgibotGdk.reset()
    manager = make_manager()

    control_lease = manager.acquire(blocking=True, initialize=True, purpose="motion")
    assert control_lease is not None
    try:
        readonly_lease = manager.acquire(
            blocking=False,
            initialize=True,
            purpose="current_pose",
        )

        assert readonly_lease is None
        assert manager.busy is True
        assert manager.active_purpose == "motion"
    finally:
        control_lease.release()


def test_session_manager_shutdown_skips_release_when_operation_is_active() -> None:
    FakeAgibotGdk.reset()
    manager = make_manager()

    lease = manager.acquire(blocking=True, initialize=True, purpose="motion")
    assert lease is not None
    try:
        shutdown = manager.shutdown(timeout=0.01)

        assert shutdown["called"] is False
        assert shutdown["success"] is False
        assert shutdown["busy"] is True
        assert shutdown["active_purpose"] == "motion"
        assert FakeAgibotGdk.release_called == 0
    finally:
        lease.release()
