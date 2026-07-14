# Changelog

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
