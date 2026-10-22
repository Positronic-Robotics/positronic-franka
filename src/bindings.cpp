#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include "robot.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_franka, m) {
  m.doc() = "Franka robot driver";

  py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
      .value("Enforce", franka::RealtimeConfig::kEnforce)
      .value("Ignore", franka::RealtimeConfig::kIgnore)
      .export_values();

  py::class_<positronic_franka::InternalImpedance>(m, "InternalImpedance",
      "Robot's built-in joint impedance controller fed by Ruckig-shaped joint position references. "
      "k_theta is the joint stiffness (7,), strictly positive, default factory-stiff; damping is "
      "managed internally.")
      .def(py::init<const std::array<double, 7>&>(),
           py::arg("k_theta") = positronic_franka::InternalImpedance{}.k_theta)
      .def_readonly("k_theta", &positronic_franka::InternalImpedance::k_theta)
      .def(py::self == py::self);

  py::class_<positronic_franka::SoftwareImpedance>(m, "SoftwareImpedance",
      "Software impedance law on the torque interface (polymetis HybridJointImpedanceControl): "
      "tau = (J^T Kx J + Kq)(q_d - q) - (J^T Kxd J + Kqd) dq + coriolis. The reference q_d moves to "
      "each target and the law pulls the arm in. Default gains are polymetis'. Otherwise "
      "all four are passed together, and each half (joint kq/kqd, Cartesian kx/kxd) is either all "
      "zero — disabled — or strictly positive, with at least one half active. "
      "max_spring_torque_rate (7,), Nm/s, caps how fast the spring term (J^T Kx J + Kq)(q_d - q) may "
      "change per joint, so a target step ramps the spring torque in rather than stepping it; damping "
      "and coriolis are not capped. Finite and strictly positive. The default is tuned for the default "
      "gains; values at or above libfranka's own 1000 Nm/s torque rate limit have no practical effect. "
      "max_reference_speed (7,), rad/s, is the speed at which q_d moves to a new target, in a straight "
      "line in joint space with all joints arriving together, so a large target jump is a slow move at "
      "any command rate. Finite and strictly positive.")
      .def(py::init<const std::array<double, 7>&, const std::array<double, 7>&>(), py::kw_only(),
           py::arg("max_spring_torque_rate") = positronic_franka::DEFAULT_MAX_SPRING_TORQUE_RATE,
           py::arg("max_reference_speed") = positronic_franka::DEFAULT_MAX_REFERENCE_SPEED)
      .def(py::init<const std::array<double, 7>&, const std::array<double, 7>&,
                    const std::array<double, 6>&, const std::array<double, 6>&,
                    const std::array<double, 7>&, const std::array<double, 7>&>(),
           py::arg("kq"), py::arg("kqd"), py::arg("kx"), py::arg("kxd"), py::kw_only(),
           py::arg("max_spring_torque_rate") = positronic_franka::DEFAULT_MAX_SPRING_TORQUE_RATE,
           py::arg("max_reference_speed") = positronic_franka::DEFAULT_MAX_REFERENCE_SPEED)
      .def_readonly("kq", &positronic_franka::SoftwareImpedance::kq)
      .def_readonly("kqd", &positronic_franka::SoftwareImpedance::kqd)
      .def_readonly("kx", &positronic_franka::SoftwareImpedance::kx)
      .def_readonly("kxd", &positronic_franka::SoftwareImpedance::kxd)
      .def_readonly("max_spring_torque_rate", &positronic_franka::SoftwareImpedance::max_spring_torque_rate)
      .def_readonly("max_reference_speed", &positronic_franka::SoftwareImpedance::max_reference_speed)
      .def(py::self == py::self);

  py::enum_<positronic_franka::GoalStatus>(m, "GoalStatus",
      "The joint move in flight. Every set_target_joints starts one; a newer target replaces an "
      "older one, and only the move in flight has a status")
      .value("NONE", positronic_franka::GoalStatus::NONE, "No move has been commanded yet")
      .value("IN_FLIGHT", positronic_franka::GoalStatus::IN_FLIGHT, "Commanded, not finished")
      .value("REACHED", positronic_franka::GoalStatus::REACHED,
             "The arm settled at the target — within tolerance and no longer moving")
      .value("ABORTED", positronic_franka::GoalStatus::ABORTED,
             "Stopped short — reflex, rejected plan, lost connection, expired deadline; see "
             "Goal.reason. The arm may still be braking when this appears");

  py::class_<positronic_franka::Goal>(m, "Goal")
      // Constructible so tests can stand one in; only the driver produces a real one.
      .def(py::init([](positronic_franka::GoalStatus status, std::string reason) {
             return positronic_franka::Goal{status, std::move(reason)};
           }),
           py::arg("status") = positronic_franka::GoalStatus::NONE, py::arg("reason") = "")
      .def_readonly("status", &positronic_franka::Goal::status)
      .def_readonly("reason", &positronic_franka::Goal::reason,
                    "Why the move stopped short, in libfranka's own words where it had any; empty "
                    "unless ABORTED")
      .def("__repr__", [](const positronic_franka::Goal& g) {
        const std::string status = py::str(py::cast(g.status));
        return g.reason.empty() ? "<Goal " + status + ">" : "<Goal " + status + ": " + g.reason + ">";
      });

  py::class_<positronic_franka::State>(m, "State")
      .def_property_readonly(
          "q",
          [](const positronic_franka::State& s) { return s.q; },
          "Measured joint positions (7,) as numpy array")
      .def_property_readonly(
          "dq",
          [](const positronic_franka::State& s) { return s.dq; },
          "Measured joint velocities (7,) as numpy array")
      .def_property_readonly(
          "q_d",
          [](const positronic_franka::State& s) { return s.q_d; },
          "Last commanded joint positions (7,) — the reference the internal controller tracks")
      .def_property_readonly(
          "tau_J",
          [](const positronic_franka::State& s) { return s.tau_J; },
          "Measured link-side joint torques (7,) as numpy array")
      .def_property_readonly(
          "tau_J_d",
          [](const positronic_franka::State& s) { return s.tau_J_d; },
          "Last commanded joint torques (7,) after rate limiting/filtering, without gravity")
      .def_property_readonly(
          "flange_pose",
          [](const positronic_franka::State& s) { return s.flange_pose; },
          "Flange pose O_T_F in robot frame as (tx,ty,tz,qw,qx,qy,qz); the end-effector frame F_T_EE "
          "that Desk sets does not move it")
      .def_property_readonly(
          "time",
          [](const positronic_franka::State& s) { return s.time; },
          "Robot controller time since start, seconds")
      .def_property_readonly(
          "error",
          [](const positronic_franka::State& s) { return s.error; },
          "1 if the robot reports an error flag, or if the control loop gets no robot state any more; else 0")
      .def_property_readonly(
          "error_message",
          [](const positronic_franka::State& s) { return s.error_message; },
          "String representation of current error flags, e.g. '[cartesian_reflex]', and the lost control link")
      .def_property_readonly(
          "ee_wrench",
          [](const positronic_franka::State& s) { return s.end_effector_wrench; },
          "External wrench on stiffness frame (Fx,Fy,Fz,Mx,My,Mz)");

  py::class_<positronic_franka::Robot>(m, "Robot")
      .def(py::init<const std::string&, franka::RealtimeConfig, double, positronic_franka::ControlMode, double>(),
           py::arg("ip"),
           py::arg("realtime_config") = franka::RealtimeConfig::kIgnore,
           py::arg("relative_dynamics_factor") = 1.0,
           py::arg("control_mode") = positronic_franka::ControlMode(positronic_franka::InternalImpedance{}),
           py::arg("control_link_timeout_s") = positronic_franka::DEFAULT_CONTROL_LINK_TIMEOUT_S,
           "control_link_timeout_s: a running control loop with no robot state for this long, in seconds, is "
           "stuck: state() reports an error, goal() aborts the move, and a stop raises RuntimeError. Default 10 s")
      .def("set_control_mode", &positronic_franka::Robot::set_control_mode,
           py::arg("mode"),
           "Apply a control mode (InternalImpedance | SoftwareImpedance): an equal mode is a no-op; a "
           "gains-only SoftwareImpedance change reaches the running torque loop without interrupting "
           "motion; any other change stops the control loop and the next motion command starts the "
           "matching one. Raises RuntimeError when it must stop a loop that gets no robot state any more")
      .def_property_readonly("control_mode", &positronic_franka::Robot::control_mode,
                             "Active control mode (InternalImpedance | SoftwareImpedance)")
      .def("state", &positronic_franka::Robot::state,
           "State snapshot from one control tick: measured q/dq/tau_J, tracked reference q_d, commanded "
           "tau_J_d, flange pose, time, errors, wrench")
      .def(
          "forward_kinematics",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& q) {
            return r.forward_kinematics(q);
          },
          py::arg("q"),
          "FK from joint vector (7,) to the flange pose (tx,ty,tz,qw,qx,qy,qz) in base frame")
      .def(
          "zero_jacobian",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& q) { return r.zero_jacobian(q); },
          py::arg("q"),
          "Zero Jacobian (6x7) of the end-effector frame F_T_EE at joint values q, with the robot's current "
          "frames — the J the SoftwareImpedance law uses")
      .def(
          "coriolis",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& q,
             const positronic_franka::Vector7d& dq) { return r.coriolis(q, dq); },
          py::arg("q"), py::arg("dq"),
          "Coriolis torques (7,) at explicit q, dq with the robot's current load configuration")
      .def(
          "inverse_kinematics",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& flange_pose_wxyz) {
            return r.inverse_kinematics(flange_pose_wxyz);
          },
          py::arg("flange_pose_wxyz"),
          "IK to the flange pose (tx,ty,tz,qw,qx,qy,qz) in base frame; returns q (7,)")
      .def(
          "inverse_kinematics",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& flange_pose_wxyz,
             const positronic_franka::Vector7d& q0) {
            return r.inverse_kinematics_q0(flange_pose_wxyz, q0);
          },
          py::arg("flange_pose_wxyz"), py::arg("q0"),
          "IK to the flange pose with initial guess q0; returns q (7,)")
      .def(
          "inverse_kinematics_with_limits",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& flange_pose_wxyz) {
            return r.inverse_kinematics_with_limits(flange_pose_wxyz);
          },
          py::arg("flange_pose_wxyz"),
          "IK for a live command to the flange pose (tx,ty,tz,qw,qx,qy,qz), from the measured joints, within "
          "the FR3 joint limits, with each joint at most 0.5 rad from the measured joints; returns q (7,)")
      .def(
          "inverse_kinematics_with_limits",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& flange_pose_wxyz,
             const positronic_franka::Vector7d& q0, double max_step) {
            return r.inverse_kinematics_with_limits(flange_pose_wxyz, q0, max_step);
          },
          py::arg("flange_pose_wxyz"), py::arg("q0"),
          py::arg("max_step") = positronic_franka::IK_MAX_JOINT_STEP,
          "IK to the flange pose from joints q0 with each joint at most max_step (rad) from q0 and within the "
          "FR3 joint limits; returns q (7,)")
      .def("set_target_joints", &positronic_franka::Robot::set_target_joints,
           py::arg("q_target"), py::arg("deadline_s") = positronic_franka::DEFAULT_MOVE_DEADLINE_S,
           "Command a joint move to target (7,) and return immediately; poll goal() for the outcome. "
           "Never blocks — waiting inside the library would stop the caller's own loop from clearing "
           "robot errors, so a reflex mid-move would never be cleared. deadline_s bounds the move: "
           "the goal ABORTS if the arm has not settled by then; pass your own where reach, gains or "
           "payload make the default wrong")
      .def("goal", &positronic_franka::Robot::goal,
           "How the move in flight is going, without blocking: Goal(status, reason)")
      .def_property_readonly("relative_dynamics_factor",
                             &positronic_franka::Robot::relative_dynamics_factor,
                             "Fixed factor scaling max vel/acc/jerk for Ruckig (0.05..1.0)")
      .def(
          "set_collision_behavior",
          py::overload_cast<
              const std::array<double, 7>&,
              const std::array<double, 7>&,
              const std::array<double, 7>&,
              const std::array<double, 7>&,
              const std::array<double, 6>&,
              const std::array<double, 6>&,
              const std::array<double, 6>&,
              const std::array<double, 6>&
          >(&positronic_franka::Robot::set_collision_behavior),
          py::arg("lower_torque_threshold_acceleration"),
          py::arg("upper_torque_threshold_acceleration"),
          py::arg("lower_torque_threshold_nominal"),
          py::arg("upper_torque_threshold_nominal"),
          py::arg("lower_force_threshold_acceleration"),
          py::arg("upper_force_threshold_acceleration"),
          py::arg("lower_force_threshold_nominal"),
          py::arg("upper_force_threshold_nominal"),
          "Set torque (7,) and force (6,) thresholds for accel/nominal and lower/upper")
      .def("set_load", &positronic_franka::Robot::set_load,
           py::arg("mass"), py::arg("F_x_Cload"), py::arg("I_x_Cload"),
           "Set tool load: mass [kg], center of mass (3,), inertia (row-major 3x3)")
      .def("stop", &positronic_franka::Robot::stop,
           "Ramp motion to a halt and join the joint control thread; a later set_target_joints restarts it. "
           "Raises RuntimeError when the loop gets no robot state any more, because such a loop never ends")
      .def("recover_from_errors", &positronic_franka::Robot::recover_from_errors,
           "Stop active control and attempt libfranka automaticErrorRecovery(); returns True if cleared. "
           "Raises RuntimeError when the loop gets no robot state any more")
      .def("get_robot_model", [](positronic_franka::Robot& r) { return r.getRobotModel(); },
           "Return URDF of the robot: libfranka's model, which ends at the flange link where the pose and the "
           "IK are, and a link 'end_effector' appended at F_T_EE, the end-effector frame that Desk sets");
}
