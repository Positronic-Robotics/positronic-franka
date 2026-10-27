# Changelog

## [Unreleased]

### Added
- `solve_damped_pose_ik` and `nullspace_projector` in `pose_ik.hpp`. `solve_damped_pose_ik` is the solver of `inverse_kinematics`, and it takes the kinematics as an argument. `tests/test_pose_ik.cpp` checks both on the FR3 flange kinematics.

### Fixed
- `inverse_kinematics` stopped 0.5 to 0.8 mm from most reachable targets near the start joints. Its pull of the joints towards zero went through `I - J⁺J` with the damped pseudo-inverse `J⁺`. With damping, that matrix moves the flange, so the pull held the error above the stop tolerance `tol`. Now the pull goes through the null-space projector of the flange Jacobian, and the solver stops within `tol`. `ik_verifier` failed its Damped IK check on about half of its targets, and now passes it. `inverse_kinematics_with_limits` does not change.

## [0.12.0] - 2026-10-24

### Added
- `Desk.operating_mode()`: the operating mode as Desk names it, for example `Execution` or `Programming`. It reads `GET /api/system/operating-mode`.
- `ProgrammingModeError`: Desk is in Programming mode and refuses FCI. A hand recovery of a joint in Desk can leave this mode, and a control box reboot clears it.

### Changed
- **Behaviour.** `prepare()` raises `ProgrammingModeError` in Programming mode. It does this after the check for a demanded recovery, and before an acknowledge, a self-test, a brake unlock or an FCI activation. Before, `prepare()` went on to those steps, and Desk refused the FCI activation with `424 ActionUnavailable WrongOperatingMode`.

## [0.11.0] - 2026-10-22

### Added
- `stiffness_point` on `SoftwareImpedance`, a keyword argument: the point (3,), in metres, where the Cartesian terms `J^T Kx J` and `J^T Kxd J` act. It is in flange axes, from the flange origin, and must be finite. A change of the point reaches the running torque loop like a gain change.
- `point_jacobian` in `pose_ik.hpp`: the zero Jacobian of a point fixed to the tool, from the tool pose and its zero Jacobian. `tests/test_pose_ik.cpp` checks it against the central differences of the point position on the FR3 flange kinematics.

### Changed
- **Behaviour.** `SoftwareImpedance` takes its Cartesian terms at `stiffness_point`, by default the flange. They were at `F_T_EE`. The default gains are polymetis', and polymetis takes them at the flange, `panda_link8`, in the DROID stack. On the FR3 with the Franka Hand configuration, `F_T_EE` is 103.4 mm past the flange. To keep the old point, pass the `F_T_EE` translation as `stiffness_point`.
- **Breaking.** `zero_jacobian(q, stiffness_point=(0, 0, 0))` returns the Jacobian at `stiffness_point`, by default the flange. It returned the Jacobian at `F_T_EE`.

## [0.10.1] - 2026-10-22

### Added
- `control_link_timeout_s` on `Robot`, a keyword argument: a running control loop that gets no robot state for this long, in seconds, is stuck. The default is `DEFAULT_CONTROL_LINK_TIMEOUT_S`, 10 s. It is far above any normal gap: at the end of a motion, libfranka calls no control callback while the robot ends the move, and on a real arm these gaps were up to 1.21 s. A value that is not finite and strictly positive raises `invalid_argument`.
- `tests/test_control_loop_watch.cpp` checks the watch with a fake control loop that ticks, stalls or exits. `tests/test_robot_control_link.cpp` checks `Robot` on a fake libfranka, `tests/fake_franka`, whose control call a test can pause. CI builds and runs both.

### Fixed
- libfranka can wait inside the control call with no time limit, for example for the reply to a Move command after the robot leaves Move mode. Then it calls no control callback. `state()` served the last state of such a loop for good, with `error` 0 and a robot time that did not advance, so the caller saw no error. Now, after `control_link_timeout_s` with no robot state, `state()` reports `error` 1 and names the lost control link in `error_message`, and `goal()` reports the move in flight as `ABORTED` with the same reason. Both also request a stop, so a loop that ticks again brakes and ends instead of driving on to a target its caller saw aborted.
- `stop()`, `set_control_mode()` and `recover_from_errors()` joined a stuck control loop for good, so the call never returned. Now they wait while the loop ticks, and raise `RuntimeError` when it gets no robot state for `control_link_timeout_s`. The stuck thread stays: nothing can end the libfranka call. The `Robot` destructor still waits for it.

## [0.10.0] - 2026-10-21

The published pose and the IK move from the end-effector frame `F_T_EE` to the flange. A pose at the flange stays valid when the tool changes. A pose at `F_T_EE` depends on the tool configuration in Desk.

### Added
- `flange_pose(O_T_EE, F_T_EE)` in `pose_ik.hpp`: `O_T_F = O_T_EE · F_T_EE⁻¹`. `tests/test_pose_ik.cpp` checks it for the Franka Hand offset and for random offsets.
- `ik_verifier` compares the published flange pose with the FK of the measured joints, and fails when they differ by more than 0.5 mm or 0.5 deg.

### Changed
- **Breaking.** `State.end_effector_pose` is replaced by `State.flange_pose`, the flange pose `O_T_F` in the base frame, from the `O_T_EE` and `F_T_EE` of the same `RobotState`. The old name is removed, so a caller that expects a pose at `F_T_EE` gets an error and not a pose at a different point. On the FR3 with the Franka Hand configuration, `F_T_EE` is 103.4 mm along the flange axis and -45 deg about it.
- **Breaking.** `forward_kinematics` returns the flange pose. `inverse_kinematics` and `inverse_kinematics_with_limits` solve for a flange pose with the flange Jacobian. Their target argument is `flange_pose_wxyz`, which replaces `target_pose_wxyz`. To command an end-effector pose `O_T_EE`, send `O_T_EE · F_T_EE⁻¹`.
- **Behaviour.** `IK_DAMPING_SINGULAR_VALUE` and the posture pull act on the flange Jacobian. Its smallest singular value differs from the one at `F_T_EE`: at the wrist-singularity pose in the tests it is 0.043 at the flange and 0.041 at the Franka Hand frame. `tests/test_pose_ik.cpp` runs its IK checks on the FR3 flange kinematics.
- `get_robot_model()` keeps the appended link `end_effector` at `F_T_EE`. The published pose and the IK are at the last link of libfranka's model, `link8` on the FR3.
- `SoftwareImpedance` keeps its Cartesian terms at `F_T_EE`, and `zero_jacobian` returns that Jacobian. `ee_wrench` stays in the stiffness frame K.

## [0.9.0] - 2026-10-18

### Added
- `max_reference_speed` on `SoftwareImpedance`, a keyword argument: a per-joint speed (7,), in rad/s, at which the reference `q_d` moves to a new target. The default is `DEFAULT_MAX_REFERENCE_SPEED`, 1.5 rad/s on every joint. `q_d` moves in a straight line in joint space, and all joints arrive together. A target near `q_d` is reached at once. A large jump is executed, slowly. This holds at any command rate, for a joint target and for an IK answer. `state().q_d` reports the moving reference. A move reports `REACHED` only after `q_d` arrives at the target. A change of the speed reaches the running torque loop like a gain change.
- `tests/test_pose_ik.cpp`, which checks the IK on FR3 kinematics with Eigen only. CI builds and runs it.
- `tests/test_trajectory_generator.cpp`, which checks the start of an `InternalImpedance` move with Eigen and the vendored Ruckig. CI builds and runs it.

### Changed
- **Behaviour.** In `SoftwareImpedance`, a target does not step `q_d` in one tick. `q_d` moves to it at `max_reference_speed`. The spring torque rate cap from 0.8.0 is unchanged.
- **Behaviour.** `inverse_kinematics_with_limits` is a new IK. It solves one bounded least-squares problem per command, from the measured joints q0: the tool as close as the bounds allow to the command, within the FR3 joint limits, and each joint at most `IK_MAX_JOINT_STEP` (0.5 rad) from q0. The bound keeps the answer on the current IK branch. It does not depend on the time since the last command. When the bound acts, the tool stops short of the command. The old IK followed the exact solution nearest q0 at any joint cost. Near the wrist singularity it turned 1-5 deg commands into opposite joint 5 and 7 steps of up to 0.66 rad.
- When the smallest singular value of the tool Jacobian at q0 is below `IK_DAMPING_SINGULAR_VALUE` (0.1, rotation rows 1 m per rad), the IK damps the whole move q - q0 by Chiaverini's law with mu_max = 0.1 / sqrt(2). Along the weak direction a tool change then moves the joints by at most the change over 0.1. Near the singularity the tool misses the command, and the joints do not jump.
- A posture row pulls the joints towards `IK_POSTURE_JOINTS` along the null space of the tool Jacobian: each command removes 2 % of the offset. It does not move the tool to first order.
- The overload with `q0` takes `max_step` (rad, default 0.5). It refuses a request where no joints are within `max_step` of `q0` and inside the FR3 limits: a negative or NaN step, or a `q0` farther than `max_step` outside a limit. `ik_verifier` passes an unbounded step.

### Fixed
- An `InternalImpedance` move starts its trajectory at rest at the robot's `q_d`, and the first tick (zero period) commands that position. The trajectory started from the measured joint speed and advanced 1 ms on the first tick, so a move that started while the arm still moved slightly could stop with `joint_motion_generator_acceleration_discontinuity`. A switch from `SoftwareImpedance` to `InternalImpedance` was the usual case.
- A stop of an `InternalImpedance` move (`stop()`, a refused target, a stall) brakes every joint to zero speed, and the arm rests where the brake ends. The stop planned a return to the position at the stop call, and Ruckig refused that plan often (`error -111`): in a simulation, 110 of 500 stops in the middle of a move. A refused stop kept the old plan, so the arm went on to the old target.
- The IK bounds each joint to the FR3 joint position limits, `FR3_JOINT_LOWER_LIMITS` and `FR3_JOINT_UPPER_LIMITS`. It used the Panda limits, which differ on every joint. Joint 6 stopped at 3.7525 rad, and the FR3 allows 4.5169 rad. Joint 4 could go up to +3.0718 rad, and the FR3 allows only -0.1518 rad. `ik_verifier` uses the same limits.

### Removed
- The OSQP dependency. The IK solves its 7-joint box-constrained problem with its own active-set solver.

## [0.8.1] - 2026-10-18

### Added
- `Desk.ensure_control()`. It takes robot control again where Desk no longer knows the session's control token, for example after a Desk tab took control and released it. Desk does not revive a token that it dropped: a later FCI call with that token answers `404 ControlTokenUnknown`, also after the other login releases control. The method never forces control.
- `ControlHeldElsewhere`, a `RuntimeError`. The context manager raises it in place of a plain `RuntimeError` while another login holds control, and `ensure_control()` raises it too. `holder` is that login. The message names it, keeps its first words, and tells the reader to release control in Desk.

## [0.8.0] - 2026-10-16

### Added
- `max_spring_torque_rate` on `SoftwareImpedance`, a keyword argument: a per-joint cap (7,), in Nm/s, on how fast the spring term `(J^T Kx J + Kq)(q_d - q)` may change. A new target steps the reference in one tick, so the spring torque stepped with it and the arm jerked at each target. Now the spring torque ramps to its new value at the cap rate, measured on the period that libfranka reports. Damping and Coriolis are not capped. Each control loop starts the ramp from zero spring torque, so the first target after a start is capped too. A stop or a stalled move sets `q_d = q`, and the spring torque then ramps to zero under the cap. A stop finishes when the arm is quiet and that ramp has ended, or after `SETTLE_TICKS_CAP` ticks. A change of the cap reaches the running torque loop like a gain change, and the ramp starts from the spring torque that the loop sent last.
- `tests/test_impedance_law.cpp`, which checks the impedance law with Eigen only. CI builds and runs it.

### Changed
- **Behaviour.** The cap is always on. The default, `DEFAULT_MAX_SPRING_TORQUE_RATE`, is `[130, 130, 130, 130, 85, 60, 35]` Nm/s for joints 1 to 7, tuned for the default gains. A caller with other gains sets its own values. Values at or above libfranka's own 1000 Nm/s torque rate limit have no practical effect.

## [0.7.2] - 2026-10-15

### Added
- `JointRecoveryRequired`, a `RuntimeError`. `prepare()` and `run_self_test()` raise it while Desk demands a recovery (`demandedRecoveries`), for example for a joint that lost its stored position. A person does this recovery at the robot with the X4 enabling device. The message names each joint as Desk's recovery screen numbers it, from 1, and `joints` holds the same numbers.

### Fixed
- `prepare()` reads `demandedRecoveries` before it acknowledges an error or runs the TD2 self-test. It acknowledged and ran the test on any active error, and the test turns the robot power off for about 30 s. Neither clears a demanded recovery, so Desk then refused the brake unlock with `424 Failed Dependency`, and a caller that retried `prepare()` turned the robot off on each try. When the self-test itself leaves a demanded recovery, `prepare()` raises the same error in place of the 424.
- A request that Desk refuses, the login included, logs the response body. Desk states the cause there, and the `HTTPError` that the call raises does not carry it.

## [0.7.1] - 2026-10-09

### Fixed
- `run_self_test()` returns when the TD2 self-test has ended, also when no test was due. It returned when `timeToTd2` read above `SELF_TEST_LEAD_SEC`. When no test is due, that is true before the test starts, so the call returned at its first read while Desk ran the test for about 30 s. A caller that opened the brakes next got `503 Service Unavailable`. `prepare()` takes this path when it runs the test to acknowledge an error. The call now waits until `timeToTd2` rises between two reads: Desk restarts the countdown when a test ends, and nothing else raises it.
- `jointPositionError` is acknowledged like the other recoverable errors. It was absent from the flag-to-id map, so `run_self_test()` acknowledged nothing and the TD2 execute that follows it answered `424 Failed Dependency` for as long as the error stood. A rig carries this error after any unclean stop, so the self-test failed exactly when it was needed. Desk's own web client names the id `JointPosition`, which is not the flag name and is why the map needed the entry rather than a derivation.

## [0.7.0] - 2026-08-04

A joint move is now commanded and observed instead of waited on: `set_target_joints` returns at once and `goal()` reports how it went.

### Added
- `goal()` — the move in flight, read without blocking: `IN_FLIGHT / REACHED / ABORTED`, with `Goal.reason` on an abort. `REACHED` means every joint is within tolerance of the commanded reference and no longer moving, in either control mode. It is a report, not a latch, and is never taken back — an arm held off target by load or friction can dip into the band at near-zero speed and report `REACHED` while settling outside it.
- `deadline_s` on `set_target_joints`, default 15 s: the goal ABORTS if the arm has not settled by then. An argument, not a constant, because reach, gains and payload decide what is reasonable. One that cannot expire (non-finite, zero, negative) raises `invalid_argument`.
- `state()` may be called while a move is in flight, as polling for the outcome requires.

### Changed
- **Breaking.** `set_target_joints` drops `asynchronous` and never blocks. For the old synchronous behaviour, poll `goal()` until it leaves `IN_FLIGHT` and raise on `ABORTED`. Blocking inside the library stopped the caller's own loop from clearing robot errors, so a reflex mid-move went uncleared and the move could never finish.
- **Breaking.** `SoftwareImpedance` steps the reference to every target and lets the impedance law pull the arm in; Ruckig shaping there was the old synchronous path. `InternalImpedance` still shapes a Ruckig trajectory.
- Every target arms a goal, streamed ones included.

### Removed
- **Breaking.** `move_to_joints(q)` — `set_target_joints` does exactly this.
- **Breaking.** `GoalStatus.SUPERSEDED` — only the move in flight has a status, so nothing ever observed the replacement.

### Fixed
- An aborted goal carries libfranka's own text (`Move command aborted: motion aborted by reflex! ["cartesian_reflex"]`) rather than a generic message; the dying thread's exception was previously printed and discarded. The stderr line stays as a second record.
- A move commanded while the robot holds an error settles ABORTED naming that error, instead of starting a control thread libfranka rejects on its first tick. Clearing stays the caller's decision.
- A move that arrives on the tick its deadline expires reports `REACHED` — arrival is tested first.
- A rejected target stops the arm, which used to keep driving toward the superseded one while the goal read ABORTED.
- `stop()`, `recover_from_errors()` and a control-mode change settle an in-flight move ABORTED, not `REACHED`.
- An older trajectory can no longer settle the goal that replaced it; each move carries an id and a settle that does not name the move in flight is ignored.

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
