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
          "end_effector_pose",
          [](const positronic_franka::State& s) { return s.end_effector_pose; },
          "End-effector pose in robot frame as (tx,ty,tz,qw,qx,qy,qz)");

  py::class_<positronic_franka::Robot>(m, "Robot")
      .def(py::init<const std::string&, franka::RealtimeConfig, double>(),
           py::arg("ip"),
           py::arg("realtime_config") = franka::RealtimeConfig::kIgnore,
           py::arg("relative_dynamics_factor") = 1.0)
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
      .def("set_target_joints", &positronic_franka::Robot::set_target_joints,
           py::arg("q_target"), py::arg("asynchronous") = true,
           "Move joints to target (7,) via Ruckig; returns immediately if asynchronous else blocks until reached")
      .def_property_readonly("relative_dynamics_factor",
                             &positronic_franka::Robot::relative_dynamics_factor,
                             "Fixed factor scaling max vel/acc/jerk for Ruckig (0.05..1.0)")
      .def("set_joint_impedance", &positronic_franka::Robot::set_joint_impedance,
           py::arg("joint_stiffness"),
           "Set joint stiffness (7,) in N·m/rad")
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
      .def("recover_from_errors", &positronic_franka::Robot::recover_from_errors,
           "Stop active control and attempt libfranka automaticErrorRecovery(); returns True if cleared");
}
