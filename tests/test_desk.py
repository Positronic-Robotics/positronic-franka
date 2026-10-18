import importlib.util
import logging
import pickle
import sys
from pathlib import Path

import pytest
import requests

# Loaded by path: the package's __init__ imports the compiled extension, which needs libfranka to build.
_DESK_PY = Path(__file__).parents[1] / 'python' / 'positronic_franka' / 'desk.py'
_spec = importlib.util.spec_from_file_location('positronic_franka_desk', _DESK_PY)
desk = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = desk  # pickle finds a class through its module
_spec.loader.exec_module(desk)

TD2_PERIOD_SEC = 86400


JOINTS = 7


class FakeResponse:
    def __init__(self, payload: dict | None = None, status_code: int = 200, text: str = ''):
        self._payload = payload or {}
        self.status_code = status_code
        self.text = text

    @property
    def ok(self) -> bool:
        return self.status_code < 400

    def raise_for_status(self) -> None:
        if not self.ok:
            raise requests.HTTPError(f'{self.status_code} Client Error', response=self)

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
        recovery_joints: tuple[int, ...] = (),
        test_demands_recovery_of: tuple[int, ...] = (),
        unlock_fails_with: int | None = None,
    ):
        """``recovery_joints`` and ``test_demands_recovery_of`` are joint indices as Desk sends them, from 0.

        ``unlock_fails_with`` is a status the brake unlock answers, after which the box answers nothing.
        """
        self.time_to_td2 = time_to_td2
        self.recovery_joints = recovery_joints
        self.test_demands_recovery_of = test_demands_recovery_of
        self.unlock_fails_with = unlock_fails_with
        self.offline = False
        self.requests: list[str] = []
        self.recoverable_errors = {flag: flag in recoverable_errors for flag in desk._ACKNOWLEDGEABLE_ERRORS}
        self.restart_to = restart_to
        self.reads_per_second = reads_per_second
        self.reads = 0
        self.test_polls_left = 0
        self.tests_run = 0
        # The control token Desk holds active, as `activeToken` reports it, and the id of each token it issued.
        self.active_token: dict | None = None
        self.token_ids: dict[str, int] = {}

    def take_control_as(self, login: str) -> None:
        """Another login takes control, as a Desk tab does. Desk forgets the token it held."""
        self.active_token = {'id': self._issue(), 'ownedBy': login}

    def release_control(self) -> None:
        self.active_token = None

    def _issue(self) -> int:
        token_id = len(self.token_ids) + 1
        self.token_ids[f'token-{token_id}'] = token_id
        return token_id

    def _known(self, headers: dict) -> bool:
        token_id = self.token_ids.get(headers.get('X-Control-Token'))
        return self.active_token is not None and token_id == self.active_token['id']

    @property
    def test_running(self) -> bool:
        return self.test_polls_left > 0

    def post(self, url: str, **kwargs) -> FakeResponse:
        return self.request('POST', url, **kwargs)

    def request(self, method: str, url: str, **kwargs) -> FakeResponse:
        if self.offline:
            raise requests.ConnectionError('the control box does not answer')
        path = url.split('://', 1)[1].split('/', 1)[1]
        if method == 'GET' and path == 'admin/api/safety/status':
            return FakeResponse(self._status())
        if method == 'GET' and path == 'admin/api/control-token':
            return FakeResponse({'activeToken': self.active_token, 'fciActive': False, 'tokenRequest': None})
        self.requests.append(f'{method} {path}')
        if method == 'POST' and path == 'admin/api/control-token/request':
            token_id = self._issue()
            if self.active_token is None:
                self.active_token = {'id': token_id, 'ownedBy': kwargs['json']['requestedBy']}
            return FakeResponse({'id': token_id, 'token': f'token-{token_id}'})
        if method == 'DELETE' and path == 'admin/api/control-token/fci':
            if not self._known(kwargs['headers']):
                return FakeResponse(status_code=404, text='{"error":"ControlTokenUnknown"}')
            return FakeResponse()
        if method == 'POST' and path == 'admin/api/login':
            return FakeResponse(status_code=401, text='{"error": "WrongCredentials"}')
        if method == 'POST' and path == 'desk/api/joints/unlock':
            if self.unlock_fails_with is not None:
                self.offline = True
                return FakeResponse(status_code=self.unlock_fails_with)
            if self.recovery_joints:
                return FakeResponse(status_code=424, text='{"error": "ActionUnavailable"}')
            raise AssertionError('the fake does not open brakes')
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
            'demandedRecoveries': {'jointPositionError': [j in self.recovery_joints for j in range(JOINTS)]},
        }
        self.reads += 1
        if self.reads % self.reads_per_second == 0:
            self.time_to_td2 -= 1
        if self.test_running:
            self.test_polls_left -= 1
            if not self.test_running:
                self.tests_run += 1
                self.time_to_td2 = self.restart_to
                self.recovery_joints = self.test_demands_recovery_of
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


def test_prepare_stops_on_a_demanded_recovery_before_any_acknowledge_or_self_test():
    # Desk sends joint index 4; its recovery screen shows it as joint 5.
    box = FakeDesk(time_to_td2=600, recoverable_errors=('jointPositionError',), recovery_joints=(4,))

    with pytest.raises(desk.JointRecoveryRequired, match='joint 5 .*X4') as raised:
        _desk_on(box).prepare()

    assert raised.value.joints == {'jointPositionError': (5,)}
    assert box.requests == []
    assert box.tests_run == 0


def test_prepare_without_a_demanded_recovery_acknowledges_and_runs_the_self_test():
    box = FakeDesk(time_to_td2=600, recoverable_errors=('jointPositionError',))
    client = _desk_on(box)
    client.open_brakes = lambda: box.requests.append('open brakes')
    client.activate_fci = lambda: box.requests.append('activate fci')

    client.prepare()

    assert box.tests_run == 1
    assert box.requests == [
        'POST admin/api/safety/recoverable-safety-errors/acknowledge?error_id=JointPosition',
        'POST admin/api/safety/td2-tests/execute',
        'open brakes',
        'activate fci',
    ]


def test_prepare_names_the_recovery_a_self_test_leaves_when_desk_refuses_the_brakes():
    box = FakeDesk(time_to_td2=600, recoverable_errors=('genericJointError',), test_demands_recovery_of=(4,))

    with pytest.raises(desk.JointRecoveryRequired, match='joint 5'):
        _desk_on(box).prepare()

    assert box.tests_run == 1
    assert box.requests[-1] == 'POST desk/api/joints/unlock'


def test_self_test_refuses_a_demanded_recovery_without_turning_the_robot_off():
    box = FakeDesk(time_to_td2=600, recovery_joints=(1, 4))

    with pytest.raises(desk.JointRecoveryRequired, match='joints 2, 5'):
        _desk_on(box).run_self_test()

    assert box.requests == []


def test_prepare_raises_an_unlock_refusal_that_is_not_424_as_desk_answered_it():
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC, unlock_fails_with=503)

    with pytest.raises(requests.HTTPError) as raised:
        _desk_on(box).prepare()

    assert raised.value.response.status_code == 503


def test_a_refused_login_logs_the_body_desk_answered(caplog):
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC)

    with caplog.at_level(logging.ERROR), pytest.raises(requests.HTTPError):
        _desk_on(box)._authenticate()

    assert 'POST /admin/api/login with 401: {"error": "WrongCredentials"}' in caplog.text


def test_a_refused_request_logs_the_body_desk_answered(caplog):
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC, recovery_joints=(4,))

    with caplog.at_level(logging.ERROR), pytest.raises(requests.HTTPError):
        _desk_on(box).open_brakes()

    assert 'POST /desk/api/joints/unlock with 424: {"error": "ActionUnavailable"}' in caplog.text


def test_joint_recovery_required_keeps_its_joints_across_a_process_boundary():
    error = desk.JointRecoveryRequired({'jointPositionError': (5,)})

    copy = pickle.loads(pickle.dumps(error))

    assert copy.joints == error.joints
    assert str(copy) == str(error)


def test_ensure_control_takes_control_again_once_the_login_that_held_it_released_it():
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC)
    client = _desk_on(box)
    client._take_control()
    box.take_control_as('safe')
    box.release_control()
    with pytest.raises(requests.HTTPError):
        client.deactivate_fci()  # Desk does not revive the token it dropped

    client.ensure_control()

    assert box.active_token['ownedBy'] == 'login'
    client.deactivate_fci()


def test_ensure_control_refuses_while_another_login_holds_control_and_takes_nothing():
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC)
    client = _desk_on(box)
    client._take_control()
    box.take_control_as('safe')
    held_by_safe = dict(box.active_token)
    sent = len(box.requests)

    with pytest.raises(desk.ControlHeldElsewhere, match="Desk login 'safe'.*release control there") as raised:
        client.ensure_control()

    assert raised.value.holder == 'safe'
    assert box.requests[sent:] == []
    assert box.active_token == held_by_safe


def test_ensure_control_takes_nothing_while_this_session_holds_control():
    box = FakeDesk(time_to_td2=TD2_PERIOD_SEC)
    client = _desk_on(box)
    client._take_control()
    sent = len(box.requests)

    client.ensure_control()

    assert box.requests[sent:] == []
    client.deactivate_fci()


def test_control_held_elsewhere_keeps_its_holder_across_a_process_boundary():
    error = desk.ControlHeldElsewhere('desk.invalid', 'safe')

    copy = pickle.loads(pickle.dumps(error))

    assert copy.holder == 'safe'
    assert str(copy) == str(error)
