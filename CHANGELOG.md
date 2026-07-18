# Changelog

## [0.6.0] - 2026-07-17

### Added
- Control modes: `InternalImpedance(k_theta)` (built-in joint impedance controller, Ruckig-shaped references — previous behavior) and `SoftwareImpedance(kq, kqd, kx, kxd)` (polymetis hybrid impedance law `tau = (J^T Kx J + Kq)(q_d - q) - (J^T Kxd J + Kqd) dq + coriolis` over the torque interface, `limit_rate` on, 100 Hz cutoff — DROID execution semantics). `Robot` applies the initial mode in its constructor; `set_control_mode()` applies a mode with the least interruption the change allows — an equal mode is a no-op, a gains-only `SoftwareImpedance` change reaches the running torque loop without interrupting motion, and any other change stops the control loop so the next motion command starts the matching one. The mode structs compare by value (Python `==` included). Under `SoftwareImpedance`, async `set_target_joints` steps the reference instantly; sync calls shape it with Ruckig, tracked by the same law. Defaults are the factory stiffness and DROID's polymetis gains.
- Expose `q_d` (commanded reference; under `SoftwareImpedance` the loop's stepped/shaped reference), `tau_J` (measured joint torques), `tau_J_d` (commanded torque after rate limiting/filtering, gravity-free), and `time` (controller clock) on `State`.
- `zero_jacobian(q)` and `coriolis(q, dq)` — the exact model terms the torque loop uses, for offline validation of a logged trace against the law.

### Removed
- `set_joint_impedance` — joint stiffness is owned by the `InternalImpedance` control mode; pass it to the `Robot` constructor or `set_control_mode()`.
- `set_cartesian_impedance` — it parameterizes the robot's internal *Cartesian* impedance controller, which this driver never activates (motion-generator sessions run in the default joint-impedance controller mode, and the torque loop bypasses internal impedance entirely), so the setting was dead configuration. If internal Cartesian control is ever wanted, it becomes a new `ControlMode` alternative owning that stiffness.

### Fixed
- A control thread that dies (reflex, exception) now wakes a blocked synchronous `set_target_joints` caller instead of leaving it deadlocked.
- A failed Ruckig replan no longer leaves a stale/default trajectory in place — evaluating one fed NaN positions into libfranka ("lowpass-filter: … NaN") and killed the control thread. The previous plan keeps playing instead.
- An asynchronous target that supersedes a synchronous goal still in flight now releases the blocked caller instead of leaving it waiting for an unrelated goal.
- An asynchronous target that overwrites a still-queued synchronous request cancels it and releases its waiter — the stale request no longer Ruckig-shapes the async target (including a request stranded by a dead control loop).

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
