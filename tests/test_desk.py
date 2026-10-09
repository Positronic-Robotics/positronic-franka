import importlib.util
from pathlib import Path

import pytest

# Loaded by path: the package's __init__ imports the compiled extension, which needs libfranka to build.
_DESK_PY = Path(__file__).parents[1] / 'python' / 'positronic_franka' / 'desk.py'
_spec = importlib.util.spec_from_file_location('positronic_franka_desk', _DESK_PY)
desk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(desk)

TD2_PERIOD_SEC = 86400


class FakeResponse:
    def __init__(self, payload: dict | None = None):
        self._payload = payload or {}

    def raise_for_status(self) -> None:
        pass

    def json(self) -> dict:
        return self._payload


class FakeDesk:
    """A control box whose TD2 runs for a few status reads and restarts the countdown when it ends."""

    TD2_DURATION_POLLS = 3

    def __init__(
        self,
        time_to_td2: int,
        recoverable_errors: tuple[str, ...] = (),
        restart_to: int = TD2_PERIOD_SEC,
        reads_per_second: int = 1,
    ):
        self.time_to_td2 = time_to_td2
        self.recoverable_errors = {flag: flag in recoverable_errors for flag in desk._ACKNOWLEDGEABLE_ERRORS}
        self.restart_to = restart_to
        self.reads_per_second = reads_per_second
        self.reads = 0
        self.test_polls_left = 0
        self.tests_run = 0

    @property
    def test_running(self) -> bool:
        return self.test_polls_left > 0

    def request(self, method: str, url: str, **kwargs) -> FakeResponse:
        path = url.split('://', 1)[1].split('/', 1)[1]
        if method == 'GET' and path == 'admin/api/safety/status':
            return FakeResponse(self._status())
        if method == 'POST' and path.startswith('admin/api/safety/recoverable-safety-errors/acknowledge'):
            self.recoverable_errors = dict.fromkeys(self.recoverable_errors, False)
            return FakeResponse()
        if method == 'POST' and path == 'admin/api/safety/td2-tests/execute':
            self.test_polls_left = self.TD2_DURATION_POLLS
            return FakeResponse()
        raise AssertionError(f'unexpected Desk request {method} {path}')

    def _status(self) -> dict:
        status = {
            'safetyControllerStatus': 'Work',
            'timeToTd2': self.time_to_td2,
            'recoverableErrors': dict(self.recoverable_errors),
        }
        self.reads += 1
        if self.reads % self.reads_per_second == 0:
            self.time_to_td2 -= 1
        if self.test_running:
            self.test_polls_left -= 1
            if not self.test_running:
                self.tests_run += 1
                self.time_to_td2 = self.restart_to
        return status


@pytest.fixture(autouse=True)
def _no_poll_wait(monkeypatch):
    monkeypatch.setattr(desk, '_POLL_INTERVAL_SEC', 0.0)


def _desk_on(box: FakeDesk) -> desk.Desk:
    client = desk.Desk('desk.invalid', 'login', 'password')
    client._session = box
    return client


def test_self_test_that_was_not_due_returns_only_after_the_test_ends():
    # The last test ended an hour ago, so no test is due. A caller runs one to acknowledge the joint error.
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC - 3600, recoverable_errors=('genericJointError',))

    _desk_on(box).run_self_test()

    assert not box.test_running
    assert box.tests_run == 1


def test_self_test_that_was_due_returns_after_the_test_ends():
    box = FakeDesk(time_to_td2=desk.SELF_TEST_LEAD_SEC - 600)

    _desk_on(box).run_self_test()

    assert not box.test_running
    assert box.tests_run == 1


def test_self_test_returns_when_the_countdown_restarts_below_its_value_at_the_execute(monkeypatch):
    monkeypatch.setattr(desk, '_SELF_TEST_TIMEOUT_SEC', 1.0)
    # A test ended a moment ago. Desk counts the new period from the start of this test, so the restart value is
    # below the countdown at the execute.
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC, restart_to=TD2_PERIOD_SEC - FakeDesk.TD2_DURATION_POLLS)

    _desk_on(box).run_self_test()

    assert not box.test_running
    assert box.tests_run == 1


def test_self_test_does_not_return_while_the_countdown_holds_between_reads():
    # Desk reports whole seconds and the reads come faster, so two reads during the test can show the same value.
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC - 3600, reads_per_second=2)

    _desk_on(box).run_self_test()

    assert not box.test_running
    assert box.tests_run == 1
