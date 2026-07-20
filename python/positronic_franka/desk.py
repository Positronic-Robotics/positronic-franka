"""Franka Desk web API client for headless brake, FCI, and self-test control.

libfranka (the ``_franka`` bindings) cannot open brakes, activate the FCI, or run the periodic TD2 safety
self-test; those live only behind Desk's HTTPS/JSON API on the robot's web port. ``Desk`` speaks that API. Used
as a context manager it takes robot control on entry and always releases it on exit, so a crashed or failing
session never strands control:

    with Desk(host, login, password) as desk:
        desk.prepare()  # self-test if due, open brakes, activate FCI
        ...             # run the arm over libfranka
"""

import base64
import contextlib
import hashlib
import logging
import time

import requests
import urllib3

logger = logging.getLogger(__name__)

# The TD2 safety self-test must run every 24h; run it proactively when the next one is due within this window
# so a session never stalls waiting for a mandatory test mid-run.
SELF_TEST_LEAD_SEC = 3600

# Unlocking/locking physically releases or engages the 7 joint brakes; Desk holds the POST open for the whole
# operation, which runs far longer than a normal request.
_BRAKE_OP_TIMEOUT_SEC = 60.0
_BRAKE_TIMEOUT_SEC = 20.0
_FCI_TIMEOUT_SEC = 15.0
_SELF_TEST_TIMEOUT_SEC = 180.0
_POLL_INTERVAL_SEC = 0.5

_CONTROL_HELD_MSG = (
    'Another session holds robot control. Open Franka Desk at https://{host}, release control there, then start again.'
)

# An active recoverable safety error puts the safety controller into Recovery, where Desk refuses every action
# (424 ActionUnavailable/RecoverableErrorActive) — including unlocking the brakes and running the self-test — until
# the error is acknowledged. Maps the flags Desk reports to the error ids its acknowledge endpoint expects.
_ACKNOWLEDGEABLE_ERRORS = {'td2Timeout': 'TD2Timeout', 'genericJointError': 'GenericJointError'}


def _acknowledgeable_errors(status: dict) -> list[str]:
    return [error_id for flag, error_id in _ACKNOWLEDGEABLE_ERRORS.items() if status['recoverableErrors'][flag]]


def _safety_error_reasons(status: dict) -> list[str]:
    reasons = status['safetyControllerStatusReason']
    return [reason for reason, active in reasons.items() if (any(active) if isinstance(active, list) else active)]


def encode_password(login: str, password: str) -> str:
    """Encode a Desk password the way the Desk web client does: base64 of the comma-joined sha256 digest bytes."""
    digest = hashlib.sha256(f'{password}#{login}@franka'.encode()).digest()
    return base64.b64encode(','.join(str(b) for b in digest).encode()).decode()


class Desk:
    def __init__(self, host: str, login: str, password: str, timeout: float = 10.0):
        self.host = host
        self._login = login
        self._password = password
        self._timeout = timeout
        self._session = requests.Session()
        self._session.verify = False
        self._control_token: str | None = None
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

    def _request(self, method: str, path: str, **kwargs) -> requests.Response:
        headers = kwargs.pop('headers', {})
        timeout = kwargs.pop('timeout', self._timeout)
        if self._control_token is not None:
            headers['X-Control-Token'] = self._control_token
        response = self._session.request(
            method, f'https://{self.host}{path}', headers=headers, timeout=timeout, **kwargs
        )
        response.raise_for_status()
        return response

    def _authenticate(self) -> None:
        encoded = encode_password(self._login, self._password)
        response = self._session.post(
            f'https://{self.host}/admin/api/login',
            json={'login': self._login, 'password': encoded},
            timeout=self._timeout,
        )
        response.raise_for_status()
        self._session.cookies.set('authorization', response.text.strip())

    def _take_control(self) -> None:
        # A request made while control is held becomes a pending token that Desk promotes to active once the holder
        # releases, silently transferring control here. Refuse up front if control is held; and if control is taken
        # in the race between that check and our request, drop the pending token rather than keep it.
        if self._token_state()['activeToken'] is not None:
            raise RuntimeError(_CONTROL_HELD_MSG.format(host=self.host))
        request = self._request('POST', '/admin/api/control-token/request', json={'requestedBy': self._login}).json()
        self._control_token = request['token']
        try:
            active = self._token_state()['activeToken']
        except Exception:
            # The POST most likely granted us the token; release it rather than strand it, then surface the
            # original error.
            with contextlib.suppress(Exception):
                self._release_control()
            raise
        if active is None or active['id'] != request['id']:
            self._control_token = None
            self._request(
                'DELETE',
                f'/admin/api/control-token/request/{request["id"]}',
                json={'token': request['token']},
                headers={'X-Control-Token': request['token']},
            )
            raise RuntimeError(_CONTROL_HELD_MSG.format(host=self.host))

    def _release_control(self) -> None:
        if self._control_token is None:
            return
        self._request('DELETE', '/admin/api/control-token', json={'token': self._control_token})
        self._control_token = None

    def _token_state(self) -> dict:
        return self._request('GET', '/admin/api/control-token').json()

    def safety_status(self) -> dict:
        return self._request('GET', '/admin/api/safety/status').json()

    def _wait_for_brakes(self, state: str) -> None:
        deadline = time.monotonic() + _BRAKE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if all(brake == state for brake in self.safety_status()['brakeState']):
                return
            time.sleep(_POLL_INTERVAL_SEC)
        raise TimeoutError(f'Brakes did not reach {state!r} within {_BRAKE_TIMEOUT_SEC}s')

    def open_brakes(self) -> None:
        self._request('POST', '/desk/api/joints/unlock', timeout=_BRAKE_OP_TIMEOUT_SEC)
        self._wait_for_brakes('Unlocked')

    def close_brakes(self) -> None:
        self._request('POST', '/desk/api/joints/lock', timeout=_BRAKE_OP_TIMEOUT_SEC)
        self._wait_for_brakes('Locked')

    def activate_fci(self) -> None:
        # Desk acknowledges the POST before FCI mode is actually up; connecting libfranka in that window is
        # refused, so wait until Desk reports FCI active.
        encoded = base64.b64encode(self._control_token.encode()).decode()
        self._request('POST', '/desk/api/system/fci', data={'token': encoded})
        deadline = time.monotonic() + _FCI_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self._token_state()['fciActive']:
                return
            time.sleep(_POLL_INTERVAL_SEC)
        raise TimeoutError(f'FCI did not activate within {_FCI_TIMEOUT_SEC}s')

    def deactivate_fci(self) -> None:
        self._request('DELETE', '/admin/api/control-token/fci', json={'token': self._control_token})

    def reboot(self) -> None:
        """Reboot the control box. Needs authentication but not robot control, works outside the context manager
        (entry is impossible while a crashed session's token is stranded), and invalidates any held control token.
        The box is unreachable for ~40s afterwards."""
        self._authenticate()
        self._request('POST', '/admin/api/reboot')
        self._control_token = None

    def run_self_test(self) -> None:
        """Acknowledge any recoverable safety error, then run the TD2 self-test. Mirrors Desk's "Acknowledge &
        Execute": once the tests are overdue the error must be acknowledged before Desk allows anything else."""
        for error_id in _acknowledgeable_errors(self.safety_status()):
            logger.info('Acknowledging recoverable safety error %s', error_id)
            self._request('POST', f'/admin/api/safety/recoverable-safety-errors/acknowledge?error_id={error_id}')
        self._request('POST', '/admin/api/safety/td2-tests/execute')
        deadline = time.monotonic() + _SELF_TEST_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self.safety_status()['timeToTd2'] > SELF_TEST_LEAD_SEC:
                return
            time.sleep(_POLL_INTERVAL_SEC)
        raise TimeoutError(f'TD2 self-test did not complete within {_SELF_TEST_TIMEOUT_SEC}s')

    def prepare(self) -> None:
        """Run the TD2 self-test if one is due or overdue, open the brakes, and activate FCI. Requires held control."""
        status = self.safety_status()
        if status['safetyControllerStatus'] == 'SafetyError':
            raise RuntimeError(
                f'Safety controller is in SafetyError ({", ".join(_safety_error_reasons(status))}); Desk refuses '
                f'every recovery action in this state. Reboot the control box (`Desk.reboot()` or the Desk web UI).'
            )
        if status['timeToTd2'] <= SELF_TEST_LEAD_SEC or _acknowledgeable_errors(status):
            logger.info('TD2 self-test due within %ds, running it now', SELF_TEST_LEAD_SEC)
            self.run_self_test()
        self.open_brakes()
        self.activate_fci()

    def __enter__(self) -> 'Desk':
        self._authenticate()
        self._take_control()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        # Each teardown step is attempted even if the previous one fails: leaving the brakes open or the control
        # token held is worse than any single failed request.
        try:
            try:
                if self._token_state()['fciActive']:
                    self.deactivate_fci()
            finally:
                if any(brake == 'Unlocked' for brake in self.safety_status()['brakeState']):
                    self.close_brakes()
        finally:
            self._release_control()
