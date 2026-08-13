import pytest

from gsa_taskflow_executor.gdk.recovery import clear_gdk_recovery_requirement


@pytest.fixture(autouse=True)
def reset_gdk_recovery_gate() -> None:
    clear_gdk_recovery_requirement(reason="test_setup")
    yield
    clear_gdk_recovery_requirement(reason="test_teardown")
