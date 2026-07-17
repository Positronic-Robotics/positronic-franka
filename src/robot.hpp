#pragma once

#include <franka/robot.h>
#include <franka/control_types.h>
#include <franka/model.h>
#include <franka/robot_state.h>

#include <array>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <variant>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ruckig/ruckig.hpp>
#include <ruckig/input_parameter.hpp>
#include <osqp.h>
#include <stdexcept>

namespace positronic_franka {

// Panda joint limits (rad/s, rad/s^2, rad/s^3)
constexpr std::array<double, 7> PANDA_BASE_VELOCITY_LIMITS = {2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
constexpr std::array<double, 7> PANDA_BASE_ACCELERATION_LIMITS = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
constexpr std::array<double, 7> PANDA_BASE_JERK_LIMITS = {5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0};
constexpr std::array<double, 7> PANDA_JOINT_LOWER_LIMITS = {
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr std::array<double, 7> PANDA_JOINT_UPPER_LIMITS = {
    2.8973, 1.7628, 2.8973, 3.0718, 2.8973, 3.7525, 2.8973};

// Common Eigen aliases
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using SpatialJacobian = Eigen::Matrix<double, 6, 7>;

// Control modes. InternalImpedance drives the robot's built-in joint impedance controller through the
// joint position motion generator with Ruckig-shaped references. SoftwareImpedance owns the impedance law
// itself: it runs the polymetis hybrid joint/Cartesian impedance over the torque interface, with async
// targets applied as instantly-stepped references (DROID execution semantics) and sync targets shaped by
// Ruckig. Defaults are the factory joint stiffness and DROID's polymetis gains respectively.
struct InternalImpedance {
  std::array<double, 7> k_theta{3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0};
};

struct SoftwareImpedance {
  std::array<double, 7> kq{40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0};
  std::array<double, 7> kqd{4.0, 6.0, 5.0, 5.0, 3.0, 2.0, 1.0};
  std::array<double, 6> kx{750.0, 750.0, 750.0, 15.0, 15.0, 15.0};
  std::array<double, 6> kxd{37.0, 37.0, 37.0, 2.0, 2.0, 2.0};
};

using ControlMode = std::variant<InternalImpedance, SoftwareImpedance>;

struct State {
  Vector7d q;
  Vector7d dq;
  // Last commanded joint positions (the reference the internal controller tracks).
  Vector7d q_d;
  // Measured link-side joint torques.
  Vector7d tau_J;
  // End-effector pose in base (robot) frame: [tx, ty, tz, qw, qx, qy, qz]
  Vector7d end_effector_pose;
  // Robot controller time since start, seconds.
  double time = 0.0;
  int error = 0;
  std::string error_message;
  // External wrench (force, torque) on end-effector frame expressed in K frame.
  Vector6d end_effector_wrench = Vector6d::Zero();
};

// Smooth trajectory generator that tracks real elapsed time.
// Uses Ruckig for trajectory computation but evaluates positions at actual
// wall-clock offsets (from libfranka's period) rather than fixed 1ms steps.
// This prevents velocity/acceleration discontinuity errors under non-RT scheduling.
class TrajectoryGenerator {
 public:
  static constexpr double NOMINAL_DT = 1.0 / 1000.0;

  explicit TrajectoryGenerator(double dynamics_factor) {
    input_.synchronization = ruckig::Synchronization::Time;
    input_.target_velocity.fill(0.0);
    input_.target_acceleration.fill(0.0);
    for (size_t i = 0; i < 7; ++i) {
      input_.max_velocity[i] = PANDA_BASE_VELOCITY_LIMITS[i] * dynamics_factor;
      input_.max_acceleration[i] = PANDA_BASE_ACCELERATION_LIMITS[i] * dynamics_factor;
      input_.max_jerk[i] = PANDA_BASE_JERK_LIMITS[i] * dynamics_factor;
    }
  }

  void initialize(const franka::RobotState& st) {
    for (size_t i = 0; i < 7; ++i) {
      input_.current_position[i] = st.q[i];
      input_.current_velocity[i] = st.dq[i];
      input_.current_acceleration[i] = 0.0;
      input_.target_position[i] = st.q[i];
    }
    replan_();
  }

  // Restart the generator from a reference position at rest (torque mode shapes segments from the
  // stepped reference, not from measured robot state).
  void reset(const Vector7d& pos) {
    for (size_t i = 0; i < 7; ++i) {
      input_.current_position[i] = pos[i];
      input_.current_velocity[i] = 0.0;
      input_.current_acceleration[i] = 0.0;
      input_.target_position[i] = pos[i];
    }
    replan_();
  }

  void set_target(const Vector7d& target) {
    for (size_t i = 0; i < 7; ++i) input_.target_position[i] = target[i];
    replan_();
  }

  void stop_at_current() {
    input_.target_position = input_.current_position;
    replan_();
  }

  // Advance trajectory by actual elapsed time and return the position.
  std::array<double, 7> step(franka::Duration period) {
    if (!planned_) return input_.current_position;

    double dt = period.toSec();
    if (dt <= 0.0) dt = NOMINAL_DT;
    cumulative_time_ += dt;

    double t = std::min(cumulative_time_, duration_);
    std::array<double, 7> pos, vel, acc;
    trajectory_.at_time(t, pos, vel, acc);

    input_.current_position = pos;
    input_.current_velocity = vel;
    input_.current_acceleration = acc;

    if (cumulative_time_ >= duration_) active_ = false;
    return pos;
  }

  bool active() const { return active_; }

 private:
  void replan_() {
    // Calculate into a scratch trajectory and swap only on success: a failed calculate must not leave a
    // default-constructed/stale trajectory in place — evaluating one feeds garbage (NaN) into libfranka
    // and kills the control thread. On failure the previous plan keeps playing.
    ruckig::Trajectory<7> next;
    auto result = otg_.calculate(input_, next);
    if (result < 0) {
      std::cerr << "Ruckig trajectory planning failed (error " << static_cast<int>(result) << ")" << std::endl;
      return;
    }
    trajectory_ = next;
    planned_ = true;
    cumulative_time_ = 0.0;
    duration_ = trajectory_.get_duration();
    active_ = true;
  }

  ruckig::Ruckig<7> otg_{NOMINAL_DT};
  ruckig::InputParameter<7> input_;
  ruckig::Trajectory<7> trajectory_;
  double cumulative_time_ = 0.0;
  double duration_ = 0.0;
  bool active_ = false;
  bool planned_ = false;
};

class Robot {
 public:
  explicit Robot(const std::string& ip,
                 franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kIgnore,
                 double relative_dynamics_factor = 1.0,
                 ControlMode control_mode = InternalImpedance{})
      : ip_(ip),
        robot_(std::make_unique<franka::Robot>(ip, realtime_config)),
        relative_dynamics_factor_(std::clamp(relative_dynamics_factor, 0.0001, 1.0)),
        control_mode_(control_mode) {
    model_ = std::make_unique<franka::Model>(robot_->loadModel());
  }

  ~Robot() {
    stop_control_loop_();
  }

  State state() {
    franka::RobotState rs = read_robot_state_();
    // Map the column-major 4x4 transform into Eigen
    Eigen::Map<const Eigen::Matrix4d> T(rs.O_T_EE.data());
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Quaterniond q(R);

    State st{};
    st.q = Eigen::Map<const Vector7d>(rs.q.data());
    st.dq = Eigen::Map<const Vector7d>(rs.dq.data());
    st.q_d = Eigen::Map<const Vector7d>(rs.q_d.data());
    st.tau_J = Eigen::Map<const Vector7d>(rs.tau_J.data());
    st.end_effector_pose << t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z();
    st.time = rs.time.toSec();
    st.error = rs.current_errors ? 1 : 0;
    if (st.error) {
      st.error_message = static_cast<std::string>(rs.current_errors);
    }
    // NOTE: This relies on the fact that we don't configure EE_T_K frame.
    st.end_effector_wrench = Eigen::Map<const Vector6d>(rs.K_F_ext_hat_K.data());
    return st;
  }

  // Stop the control loop and switch the impedance backend. The new loop starts lazily on the next
  // motion command. For InternalImpedance the joint stiffness is pushed to the robot here; SoftwareImpedance
  // gains live purely in the torque loop.
  void set_control_mode(const ControlMode& mode) {
    stop_control_loop_();
    control_mode_ = mode;
    if (const auto* internal = std::get_if<InternalImpedance>(&control_mode_)) {
      robot_->setJointImpedance(internal->k_theta);
    }
  }

  ControlMode control_mode() const { return control_mode_; }

  void set_target_joints(const Eigen::Ref<const Vector7d>& q_target,
                         bool asynchronous = true) {
    if (!control_running_.load()) {
      if (control_thread_.joinable()) {
        control_thread_.join();
      }
      stop_requested_.store(false);
      control_running_.store(true);
      control_thread_ = std::thread([this] {
        if (const auto* software = std::get_if<SoftwareImpedance>(&control_mode_)) {
          this->run_joint_torque_control_(*software);
        } else {
          this->run_joint_position_control_();
        }
      });
    }
    if (!asynchronous) {
      // Prepare synchronous wait before publishing the target to avoid races.
      std::lock_guard<std::mutex> glk(goal_mutex_);
      goal_completed_ = false;
      sync_request_next_.store(true);
    }
    {
      std::lock_guard<std::mutex> lk(target_mutex_);
      target_q_ = q_target;
      has_target_.store(true);
    }
    if (!asynchronous) {
      // Also wake when the control thread dies (reflex, exception): a dead thread can never complete
      // the goal, and finish_control_thread_ notifies after clearing control_running_.
      std::unique_lock<std::mutex> lk(goal_mutex_);
      goal_cv_.wait(lk, [&]{ return goal_completed_ || !control_running_.load(); });
    }
  }

  // Forward Kinematics: compute EE pose (tx, ty, tz, qw, qx, qy, qz) from joints q (7,)
  Vector7d forward_kinematics(
      const Eigen::Ref<const Vector7d>& q) {
    // Use current robot state for fixed frames (F_T_EE, EE_T_K), override q
    franka::RobotState st = read_robot_state_();
    st.q = to_std_array7_(q);

    const Eigen::Matrix4d T = ee_pose_matrix_(st);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Quaterniond quat(R);

    Vector7d pose;
    pose << t.x(), t.y(), t.z(), quat.w(), quat.x(), quat.y(), quat.z();
    return pose;
  }

  // Inverse Kinematics to EndEffector pose in base frame (tx, ty, tz, qw, qx, qy, qz)
  Vector7d inverse_kinematics(
      const Eigen::Ref<const Vector7d>& target_pose_wxyz,
      double tol = 1e-4, int max_iters = 150, double min_step = 1e-8, double pinv_reg = 0.03,
      double nullspace_gain = 0.002, double line_search_alpha = 1.0, double line_search_beta = 0.5,
      int line_search_max_steps = 20) {
    auto base = read_robot_state_();
    Vector7d q0 = Eigen::Map<const Vector7d>(base.q.data());
    return inverse_kinematics_q0(target_pose_wxyz, q0, tol, max_iters, min_step, pinv_reg, nullspace_gain,
                                 line_search_alpha, line_search_beta, line_search_max_steps);
  }

  Vector7d inverse_kinematics_q0(
      const Eigen::Ref<const Vector7d>& target_pose_wxyz,
      const Eigen::Ref<const Vector7d>& q0,
      double tol = 1e-4, int max_iters = 150, double min_step = 1e-8, double pinv_reg = 0.03,
      double nullspace_gain = 0.002, double line_search_alpha = 1.0, double line_search_beta = 0.5,
      int line_search_max_steps = 20) {
    // Target
    const Eigen::Vector3d t_tgt = target_pose_wxyz.head<3>();
    Eigen::Quaterniond q_tgt(target_pose_wxyz(3), target_pose_wxyz(4), target_pose_wxyz(5), target_pose_wxyz(6));
    q_tgt.normalize();
    const Eigen::Matrix3d R_tgt = q_tgt.toRotationMatrix();

    // Base state for frames (F_T_EE, EE_T_K). We'll vary q only.
    franka::RobotState base = read_robot_state_();
    franka::RobotState st = base;
    Vector7d q = q0;

    for (int it = 0; it < max_iters; ++it) {
      // Update state with current q
      st.q = to_std_array7_(q);

      // Pose and error
      const Eigen::Matrix4d T_cur = ee_pose_matrix_(st);
      const Eigen::Matrix<double, 6, 1> e = cartesian_error_(T_cur, t_tgt, R_tgt);
      const double err_norm = e.norm();
      if (err_norm < tol) break;

      // Jacobian and DLS step with nullspace bias
      const SpatialJacobian J = ee_jacobian_(st);
      const Eigen::Matrix<double, 7, 6> J_pinv = damped_pinv_(J, pinv_reg);
      const Vector7d dq_primary = -J_pinv * e;
      const Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;
      const Vector7d dq_null = N * (-nullspace_gain * std::exp(err_norm) * q);
      const Vector7d dq = dq_primary + dq_null;

      // Backtracking line search on error norm with improvement check
      double step = line_search_alpha;
      double best_err = err_norm;
      for (int ls = 0; ls < line_search_max_steps; ++ls) {
        const Vector7d q_trial = q + step * dq;
        st.q = to_std_array7_(q_trial);
        const Eigen::Matrix4d T_trial = ee_pose_matrix_(st);
        const double err_trial = cartesian_error_(T_trial, t_tgt, R_tgt).norm();
        if (err_trial < best_err) {
          best_err = err_trial;
          q = q_trial;
        }
        step *= line_search_beta;
        if (step < min_step) break;  // Stop searching further if step is too small
      }
      if (best_err >= err_norm - 1e-9) break;  // No meaningful improvement, terminate
    }
    // Print final Cartesian error (translation in mm, rotation in degrees) to stderr
    st.q = to_std_array7_(q);
    const Eigen::Matrix4d T_final = ee_pose_matrix_(st);
    const Eigen::Matrix<double, 6, 1> e_final = cartesian_error_(T_final, t_tgt, R_tgt);
    const double trans_err_mm = 1000.0 * e_final.head<3>().norm();
    const double rot_err_deg = (180.0 / M_PI) * e_final.tail<3>().norm();
    return q;
  }

  Vector7d inverse_kinematics_with_limits(
      const Eigen::Ref<const Vector7d>& target_pose_wxyz,
      double tol = 1e-4, int max_iters = 150, double min_step = 1e-8, double pinv_reg = 0.03,
      double nullspace_gain = 0.002, double line_search_alpha = 1.0, double line_search_beta = 0.5,
      int line_search_max_steps = 20) {
    auto base = read_robot_state_();
    Vector7d q0 = Eigen::Map<const Vector7d>(base.q.data());
    return inverse_kinematics_with_limits(target_pose_wxyz, q0, tol, max_iters, min_step, pinv_reg,
                                          nullspace_gain, line_search_alpha, line_search_beta,
                                          line_search_max_steps);
  }

  Vector7d inverse_kinematics_with_limits(
      const Eigen::Ref<const Vector7d>& target_pose_wxyz,
      const Eigen::Ref<const Vector7d>& q0,
      double tol = 1e-4, int max_iters = 150, double min_step = 1e-8, double pinv_reg = 0.03,
      double nullspace_gain = 0.002, double line_search_alpha = 1.0, double line_search_beta = 0.5,
      int line_search_max_steps = 20) {
    static_cast<void>(line_search_beta);
    static_cast<void>(line_search_max_steps);

    const Eigen::Vector3d t_tgt = target_pose_wxyz.head<3>();
    Eigen::Quaterniond q_tgt(target_pose_wxyz(3), target_pose_wxyz(4), target_pose_wxyz(5),
                             target_pose_wxyz(6));
    q_tgt.normalize();
    const Eigen::Matrix3d R_tgt = q_tgt.toRotationMatrix();

    franka::RobotState base = read_robot_state_();
    franka::RobotState st = base;
    Vector7d q = q0;

    const double regularization = std::max(pinv_reg, 1e-6);

    const c_int n = 7;
    const c_int m = 7;
    std::vector<c_int> A_indptr(n + 1, 0);
    std::vector<c_int> A_indices;
    std::vector<c_float> A_data;
    A_indices.reserve(n);
    A_data.reserve(n);
    for (c_int col = 0; col < n; ++col) {
      A_indptr[col] = static_cast<c_int>(A_data.size());
      A_indices.push_back(col);
      A_data.push_back(1.0);
    }
    A_indptr[n] = static_cast<c_int>(A_data.size());

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polish = 0;
    settings.max_iter = 400;
    settings.eps_abs = std::min(1e-6, tol * 0.1);
    settings.eps_rel = std::min(1e-6, tol * 0.1);
    settings.warm_start = 0;

    for (int it = 0; it < max_iters; ++it) {
      st.q = to_std_array7_(q);
      const Eigen::Matrix4d T_cur = ee_pose_matrix_(st);
      const Eigen::Matrix<double, 6, 1> e = cartesian_error_(T_cur, t_tgt, R_tgt);
      const double err_norm = e.norm();
      if (err_norm < tol)
        break;

      const SpatialJacobian J = ee_jacobian_(st);
      const Eigen::Matrix<double, 7, 7> P = J.transpose() * J +
                                            regularization * Eigen::Matrix<double, 7, 7>::Identity();

      std::vector<c_int> P_indptr(n + 1, 0);
      std::vector<c_int> P_indices;
      std::vector<c_float> P_data;
      P_indices.reserve(n * (n + 1) / 2);
      P_data.reserve(n * (n + 1) / 2);
      for (c_int col = 0; col < n; ++col) {
        P_indptr[col] = static_cast<c_int>(P_data.size());
        for (c_int row = 0; row <= col; ++row) {
          const double value = P(row, col);
          if (std::abs(value) < 1e-12)
            continue;
          P_indices.push_back(row);
          P_data.push_back(static_cast<c_float>(value));
        }
      }
      P_indptr[n] = static_cast<c_int>(P_data.size());

      c_float* A_data_raw = static_cast<c_float*>(c_malloc(A_data.size() * sizeof(c_float)));
      c_int* A_indices_raw = static_cast<c_int*>(c_malloc(A_indices.size() * sizeof(c_int)));
      c_int* A_indptr_raw = static_cast<c_int*>(c_malloc(A_indptr.size() * sizeof(c_int)));
      if (!A_data_raw || !A_indices_raw || !A_indptr_raw) {
        if (A_data_raw) c_free(A_data_raw);
        if (A_indices_raw) c_free(A_indices_raw);
        if (A_indptr_raw) c_free(A_indptr_raw);
        throw std::runtime_error("Failed to allocate OSQP constraint buffers.");
      }
      std::memcpy(A_data_raw, A_data.data(), A_data.size() * sizeof(c_float));
      std::memcpy(A_indices_raw, A_indices.data(), A_indices.size() * sizeof(c_int));
      std::memcpy(A_indptr_raw, A_indptr.data(), A_indptr.size() * sizeof(c_int));
      csc* A_csc = csc_matrix(m, n, static_cast<c_int>(A_data.size()), A_data_raw, A_indices_raw,
                              A_indptr_raw);
      if (!A_csc) {
        c_free(A_data_raw);
        c_free(A_indices_raw);
        c_free(A_indptr_raw);
        throw std::runtime_error("Failed to construct OSQP constraint matrix.");
      }

      c_float* P_data_raw = static_cast<c_float*>(c_malloc(P_data.size() * sizeof(c_float)));
      c_int* P_indices_raw = static_cast<c_int*>(c_malloc(P_indices.size() * sizeof(c_int)));
      c_int* P_indptr_raw = static_cast<c_int*>(c_malloc(P_indptr.size() * sizeof(c_int)));
      if (!P_data_raw || !P_indices_raw || !P_indptr_raw) {
        if (P_data_raw) c_free(P_data_raw);
        if (P_indices_raw) c_free(P_indices_raw);
        if (P_indptr_raw) c_free(P_indptr_raw);
        csc_spfree(A_csc);
        throw std::runtime_error("Failed to allocate OSQP Hessian buffers.");
      }
      std::memcpy(P_data_raw, P_data.data(), P_data.size() * sizeof(c_float));
      std::memcpy(P_indices_raw, P_indices.data(), P_indices.size() * sizeof(c_int));
      std::memcpy(P_indptr_raw, P_indptr.data(), P_indptr.size() * sizeof(c_int));

      csc* P_csc = csc_matrix(n, n, static_cast<c_int>(P_data.size()), P_data_raw, P_indices_raw,
                              P_indptr_raw);
      if (!P_csc) {
        c_free(P_data_raw);
        c_free(P_indices_raw);
        c_free(P_indptr_raw);
        csc_spfree(A_csc);
        throw std::runtime_error("Failed to construct OSQP Hessian matrix.");
      }

      std::vector<c_float> q_vec(n, 0.0);
      Vector7d grad = J.transpose() * e;
      for (c_int i = 0; i < n; ++i) {
        q_vec[static_cast<size_t>(i)] =
            static_cast<c_float>(grad(static_cast<Eigen::Index>(i)));
      }

      std::vector<c_float> lower(m, 0.0);
      std::vector<c_float> upper(m, 0.0);
      for (c_int i = 0; i < m; ++i) {
        const double min_bound = PANDA_JOINT_LOWER_LIMITS[static_cast<size_t>(i)] -
                                 q(static_cast<Eigen::Index>(i));
        const double max_bound = PANDA_JOINT_UPPER_LIMITS[static_cast<size_t>(i)] -
                                 q(static_cast<Eigen::Index>(i));
        lower[static_cast<size_t>(i)] = static_cast<c_float>(min_bound);
        upper[static_cast<size_t>(i)] = static_cast<c_float>(max_bound);
      }

      c_float* q_raw = static_cast<c_float*>(c_malloc(n * sizeof(c_float)));
      c_float* l_raw = static_cast<c_float*>(c_malloc(m * sizeof(c_float)));
      c_float* u_raw = static_cast<c_float*>(c_malloc(m * sizeof(c_float)));
      if (!q_raw || !l_raw || !u_raw) {
        if (q_raw) c_free(q_raw);
        if (l_raw) c_free(l_raw);
        if (u_raw) c_free(u_raw);
        csc_spfree(P_csc);
        csc_spfree(A_csc);
        throw std::runtime_error("Failed to allocate OSQP vector buffers.");
      }
      for (c_int i = 0; i < n; ++i)
        q_raw[i] = q_vec[static_cast<size_t>(i)];
      for (c_int i = 0; i < m; ++i) {
        l_raw[i] = lower[static_cast<size_t>(i)];
        u_raw[i] = upper[static_cast<size_t>(i)];
      }

      OSQPData data;
      data.n = n;
      data.m = m;
      data.P = P_csc;
      data.A = A_csc;
      data.q = q_raw;
      data.l = l_raw;
      data.u = u_raw;

      OSQPWorkspace* workspace = nullptr;
      const c_int setup_status = osqp_setup(&workspace, &data, &settings);
      if (setup_status != 0 || workspace == nullptr) {
        if (workspace) osqp_cleanup(workspace);
        c_free(q_raw);
        c_free(l_raw);
        c_free(u_raw);
        csc_spfree(P_csc);
        csc_spfree(A_csc);
        throw std::runtime_error("Failed to set up OSQP solver.");
      }

      const c_int solve_status = osqp_solve(workspace);
      if (solve_status != 0 || workspace->info == nullptr ||
          (workspace->info->status_val != OSQP_SOLVED &&
           workspace->info->status_val != OSQP_SOLVED_INACCURATE)) {
        osqp_cleanup(workspace);
        c_free(q_raw);
        c_free(l_raw);
        c_free(u_raw);
        csc_spfree(P_csc);
        csc_spfree(A_csc);
        break;
      }

      Vector7d dq = Vector7d::Zero();
      for (c_int i = 0; i < n; ++i)
        dq(static_cast<Eigen::Index>(i)) =
            static_cast<double>(workspace->solution->x[static_cast<size_t>(i)]);

      osqp_cleanup(workspace);
      c_free(q_raw);
      c_free(l_raw);
      c_free(u_raw);
      csc_spfree(P_csc);
      csc_spfree(A_csc);

      if (dq.norm() < min_step)
        break;

      double step_scale = 1.0;
      if (line_search_alpha > 0.0 && dq.norm() > line_search_alpha)
        step_scale = line_search_alpha / dq.norm();

      Vector7d q_next = q + step_scale * dq;
      for (size_t i = 0; i < 7; ++i) {
        const double lower_limit = PANDA_JOINT_LOWER_LIMITS[i];
        const double upper_limit = PANDA_JOINT_UPPER_LIMITS[i];
        q_next(static_cast<Eigen::Index>(i)) =
            std::clamp(q_next(static_cast<Eigen::Index>(i)), lower_limit, upper_limit);
      }

      if ((q_next - q).norm() < min_step)
        break;
      q = q_next;
    }

    st.q = to_std_array7_(q);
    const Eigen::Matrix4d T_final = ee_pose_matrix_(st);
    const Eigen::Matrix<double, 6, 1> e_final = cartesian_error_(T_final, t_tgt, R_tgt);
    const double trans_err_mm = 1000.0 * e_final.head<3>().norm();
    const double rot_err_deg = (180.0 / M_PI) * e_final.tail<3>().norm();
    static_cast<void>(trans_err_mm);
    static_cast<void>(rot_err_deg);
    return q;
  }

private:

  // Utilities for IK readability
  static std::array<double, 7> to_std_array7_(const Vector7d& v) {
    std::array<double, 7> a{};
    for (size_t i = 0; i < 7; ++i) a[i] = v(i);
    return a;
  }

  Eigen::Matrix4d ee_pose_matrix_(const franka::RobotState& st) const {
    const auto T_data = model_->pose(franka::Frame::kEndEffector, st);
    return Eigen::Map<const Eigen::Matrix4d>(T_data.data());
  }

  SpatialJacobian ee_jacobian_(const franka::RobotState& st) const {
    const auto J_data = model_->zeroJacobian(franka::Frame::kEndEffector, st);
    return Eigen::Map<const SpatialJacobian>(J_data.data());
  }

  static Eigen::Matrix<double, 6, 1> cartesian_error_(const Eigen::Matrix4d& T_cur,
                                                      const Eigen::Vector3d& t_tgt,
                                                      const Eigen::Matrix3d& R_tgt) {
    const Eigen::Vector3d t_cur = T_cur.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_cur = T_cur.block<3, 3>(0, 0);
    const Eigen::Vector3d e_pos = t_cur - t_tgt;
    const Eigen::Matrix3d R_rel = R_cur.transpose() * R_tgt;
    const Eigen::AngleAxisd aa(R_rel);
    const Eigen::Vector3d w = aa.angle() * aa.axis();
    const Eigen::Vector3d e_rot = -R_cur * w;
    Eigen::Matrix<double, 6, 1> e;
    e << e_pos, e_rot;
    return e;
  }

  static Eigen::Matrix<double, 7, 6> damped_pinv_(const SpatialJacobian& J, double lambda) {
    const Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    const Eigen::Matrix<double, 6, 6> A = JJt + lambda * Eigen::Matrix<double, 6, 6>::Identity();
    return J.transpose() * A.inverse();
  }

 public:
  double relative_dynamics_factor() const { return relative_dynamics_factor_; }

 private:
  void run_joint_position_control_() {
    try {
      TrajectoryGenerator traj(relative_dynamics_factor_);

      robot_->control(
        [&, this,
         first = true,
         sync_in_flight = false,
         stopping = false](const franka::RobotState& st, franka::Duration period) mutable -> franka::JointPositions {
          {
            std::lock_guard<std::mutex> lk(last_state_mutex_);
            last_state_ = std::make_unique<franka::RobotState>(st);
          }

          if (first) {
            traj.initialize(st);
            first = false;
          } else if (!stopping && stop_requested_.load()) {
            stopping = true;
            has_target_.store(false);
            traj.stop_at_current();
          }

          if (!stopping && has_target_.load()) {
            std::lock_guard<std::mutex> lk(target_mutex_);
            traj.set_target(target_q_);
            has_target_.store(false);
            sync_in_flight = sync_request_next_.exchange(false);
          }

          auto pos = traj.step(period);

          if (!traj.active()) {
            if (sync_in_flight) {
              {
                std::lock_guard<std::mutex> lk(goal_mutex_);
                goal_completed_ = true;
              }
              goal_cv_.notify_all();
              sync_in_flight = false;
            }
            if (stopping) {
              auto cmd = franka::JointPositions(pos);
              cmd.motion_finished = true;
              return cmd;
            }
          }

          return franka::JointPositions(pos);
        });
    } catch (const std::exception& e) {
      std::cerr << "Joint control thread error: " << e.what() << std::endl;
    }
    finish_control_thread_();
  }

  // Torque-interface control loop running the polymetis hybrid impedance law:
  //   tau = (J^T Kx J + Kq)(q_d - q) - (J^T Kxd J + Kqd) dq + coriolis
  // Gravity is compensated by libfranka underneath the torque command. Shares the target/stop/sync
  // machinery with the position loop; only the reference semantics differ: async targets step the
  // reference q_d instantly (DROID execution semantics), sync targets are Ruckig-shaped and tracked
  // by the same law.
  void run_joint_torque_control_(SoftwareImpedance imp) {
    const Vector7d Kq = Eigen::Map<const Vector7d>(imp.kq.data());
    const Vector7d Kqd = Eigen::Map<const Vector7d>(imp.kqd.data());
    const Vector6d Kx = Eigen::Map<const Vector6d>(imp.kx.data());
    const Vector6d Kxd = Eigen::Map<const Vector6d>(imp.kxd.data());
    try {
      TrajectoryGenerator traj(relative_dynamics_factor_);

      robot_->control(
        [&, this,
         first = true,
         sync_in_flight = false,
         shaped = false,
         stopping = false,
         stop_ticks = 0,
         ref = Vector7d(Vector7d::Zero())](const franka::RobotState& st,
                                           franka::Duration period) mutable -> franka::Torques {
          {
            std::lock_guard<std::mutex> lk(last_state_mutex_);
            last_state_ = std::make_unique<franka::RobotState>(st);
          }

          const Vector7d q = Eigen::Map<const Vector7d>(st.q.data());
          const Vector7d dq = Eigen::Map<const Vector7d>(st.dq.data());

          if (first) {
            ref = q;
            first = false;
          } else if (!stopping && stop_requested_.load()) {
            stopping = true;
            has_target_.store(false);
            shaped = false;
            ref = q;
          }

          if (!stopping && has_target_.load()) {
            std::lock_guard<std::mutex> lk(target_mutex_);
            const bool sync = sync_request_next_.exchange(false);
            if (sync) {
              traj.reset(ref);
              traj.set_target(target_q_);
              shaped = true;
              sync_in_flight = true;
            } else {
              ref = target_q_;
              shaped = false;
            }
            has_target_.store(false);
          }

          if (shaped) {
            const auto pos = traj.step(period);
            ref = Eigen::Map<const Vector7d>(pos.data());
            if (!traj.active()) {
              shaped = false;
              if (sync_in_flight) {
                {
                  std::lock_guard<std::mutex> lk(goal_mutex_);
                  goal_completed_ = true;
                }
                goal_cv_.notify_all();
                sync_in_flight = false;
              }
            }
          }

          const auto J_arr = model_->zeroJacobian(franka::Frame::kEndEffector, st);
          const SpatialJacobian J = Eigen::Map<const SpatialJacobian>(J_arr.data());
          const auto cor = model_->coriolis(st);

          const Matrix7d Kp = J.transpose() * Kx.asDiagonal() * J + Matrix7d(Kq.asDiagonal());
          const Matrix7d Kd = J.transpose() * Kxd.asDiagonal() * J + Matrix7d(Kqd.asDiagonal());
          const Vector7d tau = Kp * (ref - q) - Kd * dq + Eigen::Map<const Vector7d>(cor.data());

          std::array<double, 7> tau_arr;
          Eigen::Map<Vector7d>(tau_arr.data()) = tau;
          franka::Torques cmd(tau_arr);

          if (stopping) {
            // The spring at ref = q plus damping brings the arm to rest; finish once quiet (1 s cap).
            ++stop_ticks;
            if (dq.cwiseAbs().maxCoeff() < 0.05 || stop_ticks >= 1000) {
              cmd.motion_finished = true;
            }
          }
          return cmd;
        },
        true /* limit_rate */, 100.0 /* cutoff_frequency */);
    } catch (const std::exception& e) {
      std::cerr << "Torque control thread error: " << e.what() << std::endl;
    }
    finish_control_thread_();
  }

  // Every control-thread exit path must clear control_running_ and then wake any synchronous waiter:
  // a thread that died mid-goal (reflex, NaN, connection loss) otherwise leaves the waiter blocked forever.
  void finish_control_thread_() {
    control_running_.store(false);
    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      goal_completed_ = true;
    }
    goal_cv_.notify_all();
  }

 public:
  void set_joint_impedance(const std::array<double, 7>& joint_stiffness) {
    robot_->setJointImpedance(joint_stiffness);
  }

  void set_cartesian_impedance(const std::array<double, 6>& cartesian_stiffness) {
    robot_->setCartesianImpedance(cartesian_stiffness);
  }

  void set_collision_behavior(
      const std::array<double, 7>& lower_torque_thresholds_acceleration,
      const std::array<double, 7>& upper_torque_thresholds_acceleration,
      const std::array<double, 7>& lower_torque_thresholds_nominal,
      const std::array<double, 7>& upper_torque_thresholds_nominal,
      const std::array<double, 6>& lower_force_thresholds_acceleration,
      const std::array<double, 6>& upper_force_thresholds_acceleration,
      const std::array<double, 6>& lower_force_thresholds_nominal,
      const std::array<double, 6>& upper_force_thresholds_nominal) {
    robot_->setCollisionBehavior(
        lower_torque_thresholds_acceleration,
        upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal,
        upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration,
        upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal,
        upper_force_thresholds_nominal);
  }

  void set_load(double mass,
                const std::array<double, 3>& F_x_Cload,
                const std::array<double, 9>& I_x_Cload) {
    robot_->setLoad(mass, F_x_Cload, I_x_Cload);
  }

  std::string getRobotModel() {
    std::string urdf = robot_->getRobotModel();

    // Read F_T_EE (flange-to-end-effector transform) from current robot state.
    // This accounts for the mounted gripper/tool and is used by libfranka's
    // model->pose(kEndEffector) for FK and IK. The base URDF from
    // robot->getRobotModel() only describes kinematics up to the flange
    // (link8), so we append a fixed joint + link to complete the chain.
    auto state = read_robot_state_();
    Eigen::Map<const Eigen::Matrix4d> F_T_EE(state.F_T_EE.data());
    const Eigen::Vector3d t = F_T_EE.block<3, 1>(0, 3);
    const Eigen::Matrix3d R = F_T_EE.block<3, 3>(0, 0);

    // URDF fixed-axis RPY (extrinsic XYZ = intrinsic ZYX reversed)
    const Eigen::Vector3d zyx = R.eulerAngles(2, 1, 0);
    const double roll = zyx(2), pitch = zyx(1), yaw = zyx(0);

    // Find the flange link name dynamically (last <link name="..."> before </robot>).
    // Panda URDFs use "panda_link8", FR3 URDFs use "link8".
    std::string flange_link = "link8";
    auto robot_end = urdf.rfind("</robot>");
    if (robot_end != std::string::npos) {
      auto last_link = urdf.rfind("<link name=\"", robot_end);
      if (last_link != std::string::npos) {
        auto name_start = last_link + 12;  // length of '<link name="'
        auto name_end = urdf.find('"', name_start);
        if (name_end != std::string::npos) {
          flange_link = urdf.substr(name_start, name_end - name_start);
        }
      }
    }

    std::ostringstream snippet;
    snippet << std::setprecision(10)
            << "\n"
            << "  <!-- WARNING: This link and joint were appended by positronic-franka,\n"
            << "       NOT part of the original URDF from libfranka getRobotModel().\n"
            << "       They encode F_T_EE (flange-to-end-effector transform) read from\n"
            << "       franka::RobotState at the time of this call, so that the URDF\n"
            << "       matches the full kinematic chain used by the driver's runtime IK. -->\n"
            << "  <link name=\"end_effector\"/>\n"
            << "  <joint name=\"flange_to_end_effector\" type=\"fixed\">\n"
            << "    <parent link=\"" << flange_link << "\"/>\n"
            << "    <child link=\"end_effector\"/>\n"
            << "    <origin xyz=\"" << t.x() << " " << t.y() << " " << t.z() << "\""
            << " rpy=\"" << roll << " " << pitch << " " << yaw << "\"/>\n"
            << "  </joint>\n";

    if (robot_end != std::string::npos) {
      urdf.insert(robot_end, snippet.str());
    }
    return urdf;
  }

  bool recover_from_errors() {
    stop_control_loop_();
    stop_requested_.store(false);
    robot_->automaticErrorRecovery();
    auto state = read_robot_state_();
    return !static_cast<bool>(state.current_errors);
  }

  void stop() {
    stop_control_loop_();
  }

 private:
  franka::RobotState read_robot_state_() {
    if (control_running_.load()) {
      std::lock_guard<std::mutex> lk(last_state_mutex_);
      if (last_state_ != nullptr)
        return *last_state_;
    }
    return robot_->readOnce();
  }

  std::string ip_;
  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;
  std::thread control_thread_;
  std::atomic<bool> control_running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> has_target_{false};

  // Synchronization for synchronous set_target_joints
  std::mutex goal_mutex_;
  std::condition_variable goal_cv_;
  bool goal_completed_ = false;
  std::atomic<bool> sync_request_next_{false};

  std::mutex last_state_mutex_;
  std::unique_ptr<franka::RobotState> last_state_;

  const double relative_dynamics_factor_{1.0};
  // Written only while the control loop is idle (constructor, set_control_mode after stop); read when
  // a motion command starts the loop.
  ControlMode control_mode_;
  std::mutex target_mutex_;
  Vector7d target_q_ = Vector7d::Zero();

  void stop_control_loop_() {
    stop_requested_.store(true);
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
    control_running_.store(false);
    has_target_.store(false);
    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      goal_completed_ = true;
    }
    goal_cv_.notify_all();
    stop_requested_.store(false);
  }
};

}  // namespace positronic_franka
