#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "robot.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_franka, m) {
  m.doc() = "Franka driver stub";

  py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
      .value("Enforce", franka::RealtimeConfig::kEnforce)
      .value("Ignore", franka::RealtimeConfig::kIgnore)
      .export_values();

  py::class_<positronic_franka::InternalImpedance>(m, "InternalImpedance",
      "Robot's built-in joint impedance controller fed by Ruckig-shaped joint position references. "
      "k_theta is the joint stiffness (7,), default factory-stiff; damping is managed internally.")
      .def(py::init([](std::array<double, 7> k_theta) {
             return positronic_franka::InternalImpedance{k_theta};
           }),
           py::arg("k_theta") = positronic_franka::InternalImpedance{}.k_theta)
      .def_readwrite("k_theta", &positronic_franka::InternalImpedance::k_theta);

  py::class_<positronic_franka::SoftwareImpedance>(m, "SoftwareImpedance",
      "Software impedance law on the torque interface (polymetis HybridJointImpedanceControl): "
      "tau = (J^T Kx J + Kq)(q_d - q) - (J^T Kxd J + Kqd) dq + coriolis. Async targets step the "
      "reference instantly; sync targets are Ruckig-shaped. Defaults are DROID's polymetis gains.")
      .def(py::init([](std::array<double, 7> kq, std::array<double, 7> kqd,
                       std::array<double, 6> kx, std::array<double, 6> kxd) {
             return positronic_franka::SoftwareImpedance{kq, kqd, kx, kxd};
           }),
           py::arg("kq") = positronic_franka::SoftwareImpedance{}.kq,
           py::arg("kqd") = positronic_franka::SoftwareImpedance{}.kqd,
           py::arg("kx") = positronic_franka::SoftwareImpedance{}.kx,
           py::arg("kxd") = positronic_franka::SoftwareImpedance{}.kxd)
      .def_readwrite("kq", &positronic_franka::SoftwareImpedance::kq)
      .def_readwrite("kqd", &positronic_franka::SoftwareImpedance::kqd)
      .def_readwrite("kx", &positronic_franka::SoftwareImpedance::kx)
      .def_readwrite("kxd", &positronic_franka::SoftwareImpedance::kxd);

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
          "end_effector_pose",
          [](const positronic_franka::State& s) { return s.end_effector_pose; },
          "End-effector pose in robot frame as (tx,ty,tz,qw,qx,qy,qz)")
      .def_property_readonly(
          "time",
          [](const positronic_franka::State& s) { return s.time; },
          "Robot controller time since start, seconds")
      .def_property_readonly(
          "error",
          [](const positronic_franka::State& s) { return s.error; },
          "1 if robot reports any current error flags else 0")
      .def_property_readonly(
          "error_message",
          [](const positronic_franka::State& s) { return s.error_message; },
          "String representation of current error flags, e.g. '[cartesian_reflex]'")
      .def_property_readonly(
          "ee_wrench",
          [](const positronic_franka::State& s) { return s.end_effector_wrench; },
          "External wrench on stiffness frame (Fx,Fy,Fz,Mx,My,Mz)");

  py::class_<positronic_franka::Robot>(m, "Robot")
      .def(py::init<const std::string&, franka::RealtimeConfig, double, positronic_franka::ControlMode>(),
           py::arg("ip"),
           py::arg("realtime_config") = franka::RealtimeConfig::kIgnore,
           py::arg("relative_dynamics_factor") = 1.0,
           py::arg("control_mode") = positronic_franka::ControlMode(positronic_franka::InternalImpedance{}))
      .def("set_control_mode", &positronic_franka::Robot::set_control_mode,
           py::arg("mode"),
           "Stop the control loop and switch backend (InternalImpedance | SoftwareImpedance); the next "
           "motion command starts the matching loop")
      .def_property_readonly("control_mode", &positronic_franka::Robot::control_mode,
                             "Active control mode (InternalImpedance | SoftwareImpedance)")
      .def("state", &positronic_franka::Robot::state,
           "Returns a State with q and dq")
      .def(
          "forward_kinematics",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& q) {
            return r.forward_kinematics(q);
          },
          py::arg("q"),
          "FK from joint vector (7,) to EE pose (tx,ty,tz,qw,qx,qy,qz) in base frame")
      .def(
          "zero_jacobian",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& q) { return r.zero_jacobian(q); },
          py::arg("q"),
          "End-effector zero Jacobian (6x7) at joint values q, with the robot's current frames — the "
          "J the SoftwareImpedance law uses")
      .def(
          "coriolis",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& q,
             const positronic_franka::Vector7d& dq) { return r.coriolis(q, dq); },
          py::arg("q"), py::arg("dq"),
          "Coriolis torques (7,) at explicit q, dq with the robot's current load configuration")
      .def(
          "inverse_kinematics",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& target_pose_wxyz) {
            return r.inverse_kinematics(target_pose_wxyz);
          },
          py::arg("target_pose_wxyz"),
          "IK to EndEffector pose (tx,ty,tz,qw,qx,qy,qz) in base frame; returns q (7,)")
      .def(
          "inverse_kinematics",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& target_pose_wxyz,
             const positronic_franka::Vector7d& q0) {
            return r.inverse_kinematics_q0(target_pose_wxyz, q0);
          },
          py::arg("target_pose_wxyz"), py::arg("q0"),
          "IK with initial guess q0; returns q (7,)")
      .def(
          "inverse_kinematics_with_limits",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& target_pose_wxyz) {
            return r.inverse_kinematics_with_limits(target_pose_wxyz);
          },
          py::arg("target_pose_wxyz"),
          "IK to EndEffector pose with Panda joint limits enforced via OSQP; returns q (7,)")
      .def(
          "inverse_kinematics_with_limits",
          [](positronic_franka::Robot& r, const positronic_franka::Vector7d& target_pose_wxyz,
             const positronic_franka::Vector7d& q0) {
            return r.inverse_kinematics_with_limits(target_pose_wxyz, q0);
          },
          py::arg("target_pose_wxyz"), py::arg("q0"),
          "IK with initial guess and Panda joint limits enforced via OSQP; returns q (7,)")
      .def("set_target_joints", &positronic_franka::Robot::set_target_joints,
           py::arg("q_target"), py::arg("asynchronous") = true,
           "Move joints to target (7,) via Ruckig; returns immediately if asynchronous else blocks until reached")
      .def_property_readonly("relative_dynamics_factor",
                             &positronic_franka::Robot::relative_dynamics_factor,
                             "Fixed factor scaling max vel/acc/jerk for Ruckig (0.05..1.0)")
      .def("set_cartesian_impedance", &positronic_franka::Robot::set_cartesian_impedance,
           py::arg("cartesian_stiffness"),
           "Set Cartesian stiffness (6,) [Fx,Fy,Fz,Mx,My,Mz]")
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
           "Ramp motion to a halt and join the joint control thread; a later set_target_joints restarts it")
      .def("recover_from_errors", &positronic_franka::Robot::recover_from_errors,
           "Stop active control and attempt libfranka automaticErrorRecovery(); returns True if cleared")
      .def("get_robot_model", [](positronic_franka::Robot& r) { return r.getRobotModel(); },
           "Return URDF of the robot with F_T_EE (end-effector frame) appended for offline IK");
}
