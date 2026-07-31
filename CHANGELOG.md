# Changelog

## [0.7.0] - 2026-07-31

### Added
- `set_target_joints(q_target, deadline_s=15.0)` — the deadline bounding the move; the goal ABORTS if the arm has not settled by then. An argument, not a constant: reach, gains and payload decide what is reasonable, and only the caller knows them.
- `goal()` — the joint move in flight, read without blocking: `GoalStatus.IN_FLIGHT / REACHED / ABORTED`, with `Goal.reason` for an aborted one. `REACHED` means the arm is inside the tolerance band of the commanded reference and no longer moving, the same criterion in both control modes. It is a report, not a latch: the loop goes on driving that target until a new one arrives. It is also never taken back, so an arm whose resting point is offset by load or friction can dip into the band at near-zero speed and report REACHED once while settling outside it.

### Changed
- **Breaking.** `set_target_joints(q_target)` no longer takes `asynchronous`, and never blocks. Waiting is the caller's, as a loop that polls `goal()`. Blocking inside the library stops the caller's own loop from clearing robot errors, so a reflex mid-move goes uncleared and the move never finishes. For the old synchronous behaviour, poll `goal()` until it leaves `IN_FLIGHT` and raise on `ABORTED`.
- **Breaking.** `SoftwareImpedance` steps the reference to each target and lets the impedance law pull the arm in, which is what the mode is for. Ruckig shaping there was the old synchronous path, so a target's motion profile no longer depends on who was waiting for it. `InternalImpedance` still shapes a Ruckig trajectory.
- Every target arms a goal, streamed ones included, so a target's treatment does not depend on whether anyone is watching it.

### Removed
- **Breaking.** `move_to_joints(q)` — `set_target_joints` now does exactly this.
- **Breaking.** `GoalStatus.SUPERSEDED`. A newer target replaces an older one and only the move in flight has a status, so nothing observes the replacement.

### Fixed
- A control loop that dies now reports *why*: the goal carries libfranka's own text (`Move command aborted: motion aborted by reflex! ["cartesian_reflex"]`) instead of the generic "control loop stopped before the joint target was reached". Previously the exception was printed in the dying thread and discarded, leaving the caller to infer the cause from `state()` — which may already have recovered. The stderr line stays as a second record.
- A motion command issued while the robot holds an error no longer starts a control thread that libfranka rejects on its first tick (`command not possible in the current mode ("Reflex")`). The move settles ABORTED naming the robot's error. Clearing it stays the caller's decision — a reflex means the arm hit something.
- A rejected joint target now stops the arm. `TrajectoryGenerator` keeps the previous plan when a replan fails, so the goal reported ABORTED while the arm kept driving toward the superseded target.
- `stop()`, `recover_from_errors()` and a control-mode change no longer settle an in-flight move as `REACHED`. A teardown ends the move wherever the arm got to, so it settles ABORTED.
- An older trajectory can no longer settle the goal that replaced it. Each move carries an id, handed to the control loop with its target, and a settle that does not name the move in flight is ignored. Previously a move replaced just as its trajectory ended completed the newly armed goal as `REACHED`.

## [0.6.2] - 2026-07-26

### Added
- `Desk.reboot(wait=...)` — with `wait`, block until the control box has dropped off the network and its safety controller has settled (`Work`, a `Recovery` with an acknowledgeable error, or a `SafetyError` a reboot did not clear — which the caller re-raises as `SafetyControllerError`; ~40s), so a caller can immediately open a fresh session; raises `TimeoutError` if it never goes down or never settles. Waiting only for the web API to answer returns too early — the controller is still arming and refuses a brake-unlock (424).
- `SafetyControllerError` — raised by `Desk.prepare()` for the unrecoverable `SafetyError` controller state, carrying the active `reasons`, so callers can catch it precisely and drive the required control-box reboot. Subclasses `RuntimeError`, so existing handlers keep working.

## [0.6.1] - 2026-07-20

### Added
- `Desk.reboot()` — reboot the control box over the Desk web API. Authenticates on its own and needs no robot control, so it is usable outside the context manager (a crashed session's stranded control token makes `__enter__` refuse). The box is unreachable for ~40s afterwards.

### Fixed
- `Desk.prepare()` detects the unrecoverable `SafetyError` controller state (e.g. `nonRecoverableSafetyError`, a per-joint safety fault) up front and raises naming the active reasons and the required control-box reboot, instead of failing later with an opaque 500 from the recoverable-error acknowledge endpoint, which Desk refuses in that state.

## [0.6.0] - 2026-07-17

### Added
- Control modes: `InternalImpedance(k_theta)` (built-in joint impedance controller, Ruckig-shaped references — previous behavior) and `SoftwareImpedance(kq, kqd, kx, kxd)` (polymetis hybrid impedance law `tau = (J^T Kx J + Kq)(q_d - q) - (J^T Kxd J + Kqd) dq + coriolis` over the torque interface, `limit_rate` on, 100 Hz cutoff — DROID execution semantics). `Robot` applies the initial mode in its constructor; `set_control_mode()` applies a mode with the least interruption the change allows — an equal mode is a no-op, a gains-only `SoftwareImpedance` change reaches the running torque loop without interrupting motion, and any other change stops the control loop so the next motion command starts the matching one. The mode structs compare by value (Python `==` included). Under `SoftwareImpedance`, async `set_target_joints` steps the reference instantly; sync calls shape it with Ruckig, tracked by the same law, and block until the measured joints settle near the reference (1 s cap). Defaults are the factory stiffness and DROID's polymetis gains.
- Expose `q_d` (commanded reference; under `SoftwareImpedance` the loop's stepped/shaped reference), `tau_J` (measured joint torques), `tau_J_d` (commanded torque after rate limiting/filtering, gravity-free), and `time` (controller clock) on `State` — all fields reported from the same control tick.
- `zero_jacobian(q)` and `coriolis(q, dq)` — the exact model terms the torque loop uses, for offline validation of a logged trace against the law.
- Mode gains are validated at construction and read-only afterwards: `k_theta` strictly positive; `SoftwareImpedance` takes either no gains (DROID's deployed values) or all four together, each half (joint `kq`/`kqd`, Cartesian `kx`/`kxd`) entirely zero — disabled — or strictly positive, with at least one half active. `set_target_joints` rejects non-finite targets.

### Removed
- `set_joint_impedance` — joint stiffness is owned by the `InternalImpedance` control mode; pass it to the `Robot` constructor or `set_control_mode()`.
- `set_cartesian_impedance` — it parameterizes the robot's internal *Cartesian* impedance controller, which this driver never activates (motion-generator sessions run in the default joint-impedance controller mode, and the torque loop bypasses internal impedance entirely), so the setting was dead configuration. If internal Cartesian control is ever wanted, it becomes a new `ControlMode` alternative owning that stiffness.

### Fixed
- A control thread that dies (reflex, exception) mid-goal now wakes a blocked synchronous `set_target_joints` caller, which raises instead of deadlocking or returning as if the target was reached.
- A failed Ruckig replan no longer leaves a stale/default trajectory in place — evaluating one fed NaN positions into libfranka ("lowpass-filter: … NaN") and killed the control thread. The previous plan keeps playing instead.
- An asynchronous target that supersedes a synchronous goal — still queued or already in flight — cancels it and releases the blocked caller instead of leaving it waiting for an unrelated goal.
- A target stranded by a dead control loop is cleared before the replacement loop starts, so its first ticks cannot move toward a stale goal.
- A synchronous target whose trajectory plan is rejected raises instead of reporting the goal reached at the old reference.

## [0.5.0] - 2026-07-10

### Added
- Expose `Robot.stop()` — ramp motion to a halt and join the joint control thread, so a session can stop control cleanly before deactivating the FCI.
- `positronic_franka.desk.Desk` — a Franka Desk web API client for headless control the FCI cannot perform: taking robot control, opening/closing the brakes, activating/deactivating the FCI, and running the TD2 safety self-test (acknowledging the overdue-test safety error first, the way Desk's "Acknowledge & Execute" does). Used as a context manager it takes control on entry and always releases it on exit, and refuses (rather than force-seizes) control held by another session.

## [0.4.0] - 2026-03-17

### Added
- Expose `get_robot_model()` — returns the robot's URDF with `F_T_EE` (flange-to-end-effector transform) baked in as a fixed joint, so the URDF matches the full kinematic chain used by the driver's runtime IK.

## [0.3.1] - 2026-02-17

### Added
- Expose `error_message` on `State` — string description of current error flags (e.g. `[cartesian_reflex]`)

### Fixed
- Replace fixed-timestep Ruckig updates with cumulative wall-clock timing (`TrajectoryGenerator`). Prevents velocity/acceleration discontinuity errors when control thread doesn't run at exactly 1kHz.

## [0.3.0] - 2025-11-05

### Changed
- **BREAKING**: Package now installs under `positronic_franka` namespace instead of `positronic.drivers.roboarm`
- This eliminates namespace package complexity and makes the package structure cleaner and more maintainable
- Import path changed from `from positronic.drivers.roboarm import _franka` to `import positronic_franka._franka`

## [0.2.2] - 2025-10-23

### Fixed
- Normalise quaternion before solving IK. Minor performance optimisation.

## [0.2.1] - 2025-10-12

### Added
- Implement IK that respects robot limits in a hard way.

### Fixed
- Fix control thread restart after reflex abort

## [0.2.0] - 2025-10-07

### Added
- Extend `State` with `error` flag and `ee_wrench` vector.
