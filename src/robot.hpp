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
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ruckig/ruckig.hpp>
#include <ruckig/input_parameter.hpp>
#include <osqp.h>

namespace positronic_franka {

// Panda joint limits (rad/s, rad/s^2, rad/s^3)
constexpr std::array<double, 7> PANDA_BASE_VELOCITY_LIMITS = {2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
constexpr std::array<double, 7> PANDA_BASE_ACCELERATION_LIMITS = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
constexpr std::array<double, 7> PANDA_BASE_JERK_LIMITS = {5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0};
constexpr std::array<double, 7> PANDA_JOINT_LOWER_LIMITS = {
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr std::array<double, 7> PANDA_JOINT_UPPER_LIMITS = {
    2.8973, 1.7628, 2.8973, 3.0718, 2.8973, 3.7525, 2.8973};

// Arrival detection, shared by both control loops: the measured joints are within these tolerances of
// the commanded reference and no longer moving.
constexpr double SETTLE_POSITION_TOLERANCE = 0.05;  // rad
constexpr double SETTLE_VELOCITY_TOLERANCE = 0.05;  // rad/s
// How long the arm may make NO PROGRESS toward the target before the move is given up as stalled
// (1 kHz ticks, so 1 s). It bounds time spent going nowhere, NOT the move: progress resets the count,
// so an arm still closing on the target is never given up on however slowly it converges — only one
// actually held short (contact, a joint limit) trips it. A stalled move ABORTS; it never reports
// arrival, because it did not arrive.
constexpr int STALL_TICKS_CAP = 1000;
// The joint error must shrink by at least this much to count as progress. Far below any real
// convergence — a move creeping at a tenth of the rest threshold still closes 500x this in a second —
// and far above the noise on a joint encoder, so it separates "converging slowly" from "not moving".
constexpr double STALL_PROGRESS_EPSILON = 1e-4;  // rad
// The whole move's deadline, whatever it is doing. The progress rule alone bounds a move only at
// `distance / STALL_PROGRESS_EPSILON` seconds — 1 rad of travel buys ~9500 resets, so an arm making
// microscopic progress could hold a caller for hours. This is the backstop: generous next to any real
// move (the impedance spring settles in seconds, a full-range Ruckig move at 0.2 dynamics in ~10),
// so it never cuts a legitimate one short, and it turns the worst case from hours into a minute.
constexpr int MOVE_TICKS_CAP = 60000;  // 1 kHz ticks — 60 s

// Common Eigen aliases
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using SpatialJacobian = Eigen::Matrix<double, 6, 7>;

// Control modes. InternalImpedance drives the robot's built-in joint impedance controller through the
// joint position motion generator with Ruckig-shaped references. SoftwareImpedance owns the impedance law
// itself: it runs the polymetis hybrid joint/Cartesian impedance over the torque interface, with every
// target applied as an instantly-stepped reference (DROID execution semantics). Defaults are the factory
// joint stiffness and DROID's polymetis gains respectively.
template <size_t N>
bool all_zero(const std::array<double, N>& v) {
  for (double x : v) {
    if (x != 0.0) return false;
  }
  return true;
}

template <size_t N>
bool all_positive(const std::array<double, N>& v) {
  for (double x : v) {
    if (!(x > 0.0) || !std::isfinite(x)) return false;  // NaN fails the comparison, inf fails isfinite
  }
  return true;
}

// A gain space is either disabled (stiffness and damping all zero) or a damped spring (both strictly
// positive); anything in between is a configuration error.
template <size_t N>
void validate_half(const std::array<double, N>& k, const std::array<double, N>& kd, const char* name) {
  if (all_zero(k) && all_zero(kd)) return;
  if (all_positive(k) && all_positive(kd)) return;
  throw std::invalid_argument(std::string(name) +
                              " stiffness and damping must be either all zero (half disabled) or strictly positive");
}

struct InternalImpedance {
  std::array<double, 7> k_theta{3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0};

  InternalImpedance() = default;
  explicit InternalImpedance(const std::array<double, 7>& k_theta_in) : k_theta(k_theta_in) {
    if (!all_positive(k_theta)) throw std::invalid_argument("k_theta must be finite and strictly positive");
  }
};

// Gain defaults are the metadata defaults of DROID's polymetis deployment — fairo
// polymetis/conf/robot_client/franka_hardware.yaml, loaded by DROID's `launch_robot.py
// robot_client=franka_hardware` — i.e. the exact configuration HybridJointImpedanceControl runs with
// in the DROID data-collection stack.
struct SoftwareImpedance {
  std::array<double, 7> kq{40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0};
  std::array<double, 7> kqd{4.0, 6.0, 5.0, 5.0, 3.0, 2.0, 1.0};
  std::array<double, 6> kx{750.0, 750.0, 750.0, 15.0, 15.0, 15.0};
  std::array<double, 6> kxd{37.0, 37.0, 37.0, 2.0, 2.0, 2.0};

  SoftwareImpedance() = default;
  SoftwareImpedance(const std::array<double, 7>& kq_in, const std::array<double, 7>& kqd_in,
                    const std::array<double, 6>& kx_in, const std::array<double, 6>& kxd_in)
      : kq(kq_in), kqd(kqd_in), kx(kx_in), kxd(kxd_in) {
    validate_half(kq, kqd, "joint (kq/kqd)");
    validate_half(kx, kxd, "Cartesian (kx/kxd)");
    if (all_zero(kq) && all_zero(kx)) {
      throw std::invalid_argument("at least one of the joint or Cartesian halves must be active");
    }
  }
};

inline bool operator==(const InternalImpedance& a, const InternalImpedance& b) {
  return a.k_theta == b.k_theta;
}

inline bool operator==(const SoftwareImpedance& a, const SoftwareImpedance& b) {
  return a.kq == b.kq && a.kqd == b.kqd && a.kx == b.kx && a.kxd == b.kxd;
}

using ControlMode = std::variant<InternalImpedance, SoftwareImpedance>;

// How the joint move now in flight is going. Every `set_target_joints` starts one.
//
// Pollable, never blocking, because a caller whose own loop is the thing that clears robot errors
// cannot afford to wait inside the library: a reflex firing during a blocking move is seen by
// nobody, so it is never cleared and the move never ends. Waiting is the caller's to write, in a
// loop that keeps doing its other work.
//
// A newer target simply replaces an older one — the arm goes where the caller last asked. The
// replaced move is not reported: only the move in flight has a status, so a caller that cares about
// arrival sends one target and waits for it.
enum class GoalStatus {
  // No move has been commanded yet on this Robot.
  NONE,
  // Commanded, not finished.
  IN_FLIGHT,
  // The arm settled at the commanded target.
  REACHED,
  // The move stopped short — a reflex, a rejected plan, a lost connection. `Goal::reason` says which.
  ABORTED,
};

struct Goal {
  GoalStatus status = GoalStatus::NONE;
  // Why the move stopped short — libfranka's own text where it had any. Empty unless ABORTED.
  std::string reason;
};

struct State {
  Vector7d q;
  Vector7d dq;
  // Last commanded joint positions (the reference the internal controller tracks).
  Vector7d q_d;
  // Measured link-side joint torques.
  Vector7d tau_J;
  // Last commanded joint torques (after rate limiting/filtering), without gravity.
  Vector7d tau_J_d;
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

  // Returns whether the plan to the new target was accepted; on failure the previous plan keeps playing.
  bool set_target(const Vector7d& target) {
    for (size_t i = 0; i < 7; ++i) input_.target_position[i] = target[i];
    return replan_();
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
  bool replan_() {
    // Calculate into a scratch trajectory and swap only on success: a failed calculate must not leave a
    // default-constructed/stale trajectory in place — evaluating one feeds garbage (NaN) into libfranka
    // and kills the control thread. On failure the previous plan keeps playing.
    ruckig::Trajectory<7> next;
    auto result = otg_.calculate(input_, next);
    if (result < 0) {
      std::cerr << "Ruckig trajectory planning failed (error " << static_cast<int>(result) << ")" << std::endl;
      return false;
    }
    trajectory_ = next;
    planned_ = true;
    cumulative_time_ = 0.0;
    duration_ = trajectory_.get_duration();
    active_ = true;
    return true;
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
      : robot_(std::make_unique<franka::Robot>(ip, realtime_config)),
        relative_dynamics_factor_(std::clamp(relative_dynamics_factor, 0.0001, 1.0)) {
    model_ = std::make_unique<franka::Model>(robot_->loadModel());
    // Force-apply: the equality guard in set_control_mode must not skip pushing k_theta when the
    // requested mode happens to equal the default — the robot may persist stiffness from past sessions.
    apply_control_mode_(control_mode);
  }

  ~Robot() {
    stop_control_loop_();
  }

  State state() {
    auto [rs, software_ref] = read_state_snapshot_();
    // Map the column-major 4x4 transform into Eigen
    Eigen::Map<const Eigen::Matrix4d> T(rs.O_T_EE.data());
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Quaterniond q(R);

    State st{};
    st.q = Eigen::Map<const Vector7d>(rs.q.data());
    st.dq = Eigen::Map<const Vector7d>(rs.dq.data());
    // Under torque control the reference lives in the software loop, not in the robot's q_d; report
    // whichever reference the active controller tracks.
    st.q_d = software_ref ? *software_ref : Eigen::Map<const Vector7d>(rs.q_d.data());
    st.tau_J = Eigen::Map<const Vector7d>(rs.tau_J.data());
    st.tau_J_d = Eigen::Map<const Vector7d>(rs.tau_J_d.data());
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

  // Apply a control mode with the least interruption the change allows: an equal mode is a no-op; a
  // gains-only SoftwareImpedance change is handed to the running torque loop without interrupting motion;
  // any other change stops the control loop (clearing the pending target) and the next motion command
  // starts the matching loop. k_theta is pushed to the robot (the internal controller reads robot-side
  // config); software gains live purely in the torque loop.
  void set_control_mode(const ControlMode& mode) {
    if (mode == control_mode_) return;
    apply_control_mode_(mode);
  }

  ControlMode control_mode() const { return control_mode_; }

  // The move in flight, read without blocking.
  Goal goal() {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    return Goal{goal_status_, goal_error_};
  }

  // Command a joint move and return immediately. Poll `goal()` for IN_FLIGHT / REACHED / ABORTED.
  //
  // One behaviour, whatever the control mode and whoever is calling: publish the target, arm the
  // goal, return. How the arm gets there is the mode's business — `InternalImpedance` shapes a
  // Ruckig trajectory, `SoftwareImpedance` steps the reference and lets the impedance law pull the
  // arm in (DROID execution semantics) — but both report the same way, and neither waits.
  void set_target_joints(const Eigen::Ref<const Vector7d>& q_target) {
  // Start the control loop if it is not running, then publish `q_target` to it. Every target arms
  // the goal, so the move reports IN_FLIGHT until the loop settles it — one path, because a target's
  // treatment must not depend on whether anyone happens to be watching it.
    // Reject garbage before it reaches a control loop: a torque-mode target becomes the reference
    // verbatim, so a NaN here would become NaN torques.
    if (!q_target.allFinite()) {
      throw std::invalid_argument("q_target must be finite");
    }
    std::uint64_t goal_id = 0;
    if (!control_running_.load()) {
      if (control_thread_.joinable()) {
        control_thread_.join();
      }
      // Arm here, after the join and before the launch below, and the goal has exactly one loop that
      // can settle it. Arming any later would leave a window where the loop dies before the goal
      // exists: the abort finds nothing in flight and is dropped, and the goal armed afterwards then
      // belongs to a dead thread, so a poller waits on a move nothing will ever settle. Arming any
      // earlier would expose it to the loop being replaced — the join is what guarantees that one has
      // finished settling, so it cannot abort the move its successor is about to run.
      goal_id = arm_goal_();
      // Starting a loop issues a Move command, and the robot refuses one while it holds an error
      // ("command not possible in the current mode"), so the fresh thread would die on its first tick.
      // Report the robot's own error instead of spawning it. Clearing is the caller's call, not this
      // library's: a reflex means the arm hit something, and only the caller knows whether resuming is
      // safe — the goal aborts with the robot's own text so the caller sees why.
      // Best effort: if the state read itself fails, fall through and let the loop report whatever is
      // wrong, rather than turning a guard into a new failure mode.
      std::string robot_errors;
      try {
        if (const auto errors = read_robot_state_().current_errors; static_cast<bool>(errors)) {
          robot_errors = static_cast<std::string>(errors);
        }
      } catch (const std::exception&) {  // NOLINT(bugprone-empty-catch)
      }
      if (!robot_errors.empty()) {
        fail_goal_(goal_id, "robot reports an error, recover before commanding a move: " + robot_errors);
        return;
      }
      stop_requested_.store(false);
      // A target or sync request stranded by a dead loop must not feed the replacement loop's first
      // ticks — those can run before this command publishes its own target below. Anything queued here
      // is stale: no loop consumed it, and the caller's publish comes after (callers are serialized).
      has_target_.store(false);
      {
        std::lock_guard<std::mutex> lk(target_mutex_);
        target_goal_id_ = 0;
      }
      control_running_.store(true);
      // Snapshot the mode here: the caller is GIL-serialized with set_control_mode, while the thread
      // body would race a concurrent gains handoff writing control_mode_.
      const ControlMode mode = control_mode_;
      try {
        control_thread_ = std::thread([this, mode] {
          if (const auto* software = std::get_if<SoftwareImpedance>(&mode)) {
            this->run_joint_torque_control_(*software);
          } else {
            this->run_joint_position_control_();
          }
        });
      } catch (...) {
        // No thread means nothing will ever clear control_running_ or settle the goal armed above.
        control_running_.store(false);
        fail_goal_(goal_id, "could not start the control loop");
        throw;
      }
    } else {
      // The loop is ALREADY running, which is every move after the first. There is no join to order
      // against here: the loop that will consume this target is the one already running, so arming
      // immediately before publishing is the whole requirement. If that loop dies in the window,
      // `finish_control_thread_` settles the goal armed here as ABORTED with the loop's own reason.
      goal_id = arm_goal_();
    }
    {
      std::lock_guard<std::mutex> lk(target_mutex_);
      target_q_ = q_target;
      target_goal_id_ = goal_id;
      has_target_.store(true);
    }
    // The loop can exit between the `control_running_` read above and this publish: its
    // `finish_control_thread_` then settles whatever goal was in flight BEFORE this one was armed,
    // and the target just published belongs to a thread that is gone — leaving `goal()` IN_FLIGHT
    // with nothing left that could ever settle it. Re-read after publishing and abort the goal
    // ourselves. `control_running_` only goes false when a loop ends and only back to true when a
    // caller starts one (callers are serialized), so a false read here is final, not a race.
    // `settle_goal_` ignores a goal already settled, so a `finish_control_thread_` landing in
    // between wins with its own, better reason.
    if (!control_running_.load()) {
      fail_goal_(goal_id, "the control loop stopped before it took up the target");
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

  // Model terms at explicit joint values, using the connected robot's current frames and load
  // configuration. These mirror the exact terms the SoftwareImpedance loop uses, so logged traces can
  // be validated offline against the implemented law.
  SpatialJacobian zero_jacobian(const Eigen::Ref<const Vector7d>& q) {
    franka::RobotState st = read_robot_state_();
    st.q = to_std_array7_(q);
    return ee_jacobian_(st);
  }

  Vector7d coriolis(const Eigen::Ref<const Vector7d>& q, const Eigen::Ref<const Vector7d>& dq) {
    franka::RobotState st = read_robot_state_();
    st.q = to_std_array7_(q);
    st.dq = to_std_array7_(dq);
    const auto cor = model_->coriolis(st);
    return Eigen::Map<const Vector7d>(cor.data());
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
    std::string thread_error;
    try {
      TrajectoryGenerator traj(relative_dynamics_factor_);

      robot_->control(
        [&, this,
         first = true,
         goal_in_flight = std::uint64_t{0},
         stall_ticks = 0,
         move_ticks = 0,
         best_err = std::numeric_limits<double>::infinity(),
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
            const bool planned = traj.set_target(target_q_);
            has_target_.store(false);
            goal_in_flight = std::exchange(target_goal_id_, 0);
            stall_ticks = 0;
            move_ticks = 0;
            best_err = std::numeric_limits<double>::infinity();
            // A rejected plan must not let the old trajectory finish this goal as if reached.
            if (!planned) {
              fail_goal_(goal_in_flight, "joint target rejected by the trajectory planner");
              goal_in_flight = 0;
            }
          }

          auto pos = traj.step(period);

          // Arrival is PHYSICAL and the criterion is the same in both control modes: the arm is at
          // the commanded reference and no longer moving. An exhausted reference is not arrival —
          // the internal controller tracks closely but not exactly, and the torque loop has no
          // trajectory to exhaust at all.
          //
          // Here it is gated on the trajectory being done, which the torque loop needs no equivalent
          // of: a Ruckig move STARTS with the reference at the current pose and the arm at rest, so
          // an ungated check would report REACHED on the first tick of every move. A stepped
          // reference starts far from the arm, so the same check is safe unguarded there.
          if (goal_in_flight != 0 && !traj.active() &&
              settle_on_arrival_(goal_in_flight,
                                 arrival_(Eigen::Map<const Vector7d>(pos.data()),
                                          Eigen::Map<const Vector7d>(st.q.data()),
                                          Eigen::Map<const Vector7d>(st.dq.data()), best_err, stall_ticks,
                                          move_ticks))) {
            goal_in_flight = 0;
          }

          if (!traj.active() && stopping) {
            auto cmd = franka::JointPositions(pos);
            cmd.motion_finished = true;
            return cmd;
          }

          return franka::JointPositions(pos);
        });
    } catch (const std::exception& e) {
      // Printed as well as reported: a streamed target arms no goal, so for those this line is the
      // only record that the loop died and why.
      std::cerr << "Joint control thread error: " << e.what() << std::endl;
      thread_error = e.what();
    }
    finish_control_thread_(std::move(thread_error));
  }

  // Torque-interface control loop running the polymetis hybrid impedance law:
  //   tau = (J^T Kx J + Kq)(q_d - q) - (J^T Kxd J + Kqd) dq + coriolis
  // Gravity is compensated by libfranka underneath the torque command. Shares the target/stop/goal
  // machinery with the position loop; only the reference semantics differ: every target steps the
  // reference q_d instantly and the law pulls the arm in (DROID execution semantics), where the
  // position loop shapes a Ruckig trajectory.
  void run_joint_torque_control_(SoftwareImpedance imp) {
    Vector7d Kq = Eigen::Map<const Vector7d>(imp.kq.data());
    Vector7d Kqd = Eigen::Map<const Vector7d>(imp.kqd.data());
    Vector6d Kx = Eigen::Map<const Vector6d>(imp.kx.data());
    Vector6d Kxd = Eigen::Map<const Vector6d>(imp.kxd.data());
    std::string thread_error;
    try {
      robot_->control(
        [&, this,
         first = true,
         goal_in_flight = std::uint64_t{0},
         stall_ticks = 0,
         move_ticks = 0,
         best_err = std::numeric_limits<double>::infinity(),
         stopping = false,
         stop_ticks = 0,
         ref = Vector7d(Vector7d::Zero())](const franka::RobotState& st,
                                           franka::Duration /*period*/) mutable -> franka::Torques {
          const Vector7d q = Eigen::Map<const Vector7d>(st.q.data());
          const Vector7d dq = Eigen::Map<const Vector7d>(st.dq.data());

          if (first) {
            ref = q;
            first = false;
          } else if (!stopping && stop_requested_.load()) {
            stopping = true;
            has_target_.store(false);
            ref = q;
          }

          // No trajectory here, by design: the reference STEPS to the target and the impedance law
          // is what moves the arm (DROID execution semantics, which is what this mode exists for).
          // Shaping it with Ruckig was the old sync path, and having two profiles behind one flag is
          // what made a target's motion depend on who was waiting for it.
          if (!stopping && has_target_.load()) {
            std::lock_guard<std::mutex> lk(target_mutex_);
            ref = target_q_;
            goal_in_flight = std::exchange(target_goal_id_, 0);
            stall_ticks = 0;
            move_ticks = 0;
            best_err = std::numeric_limits<double>::infinity();
            has_target_.store(false);
          }

          if (has_new_gains_.load()) {
            std::lock_guard<std::mutex> lk(target_mutex_);
            Kq = Eigen::Map<const Vector7d>(new_gains_.kq.data());
            Kqd = Eigen::Map<const Vector7d>(new_gains_.kqd.data());
            Kx = Eigen::Map<const Vector6d>(new_gains_.kx.data());
            Kxd = Eigen::Map<const Vector6d>(new_gains_.kxd.data());
            has_new_gains_.store(false);
          }

          // The same arrival rule the position loop uses. No gate on a trajectory is needed: the
          // reference stepped away from the arm when the target arrived, so the check cannot pass
          // until the spring has actually pulled the joints in.
          if (goal_in_flight != 0 &&
              settle_on_arrival_(goal_in_flight, arrival_(ref, q, dq, best_err, stall_ticks, move_ticks))) {
            goal_in_flight = 0;
          }

          // Publish the state and this tick's final reference in one critical section, so a state()
          // snapshot can never pair them from different ticks.
          {
            std::lock_guard<std::mutex> lk(last_state_mutex_);
            last_state_ = std::make_unique<franka::RobotState>(st);
            last_software_ref_ = ref;
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
            // The spring at ref = q plus damping brings the arm to rest; finish once quiet (capped).
            ++stop_ticks;
            if (dq.cwiseAbs().maxCoeff() < SETTLE_VELOCITY_TOLERANCE || stop_ticks >= SETTLE_TICKS_CAP) {
              cmd.motion_finished = true;
            }
          }
          return cmd;
        },
        true /* limit_rate */, 100.0 /* cutoff_frequency */);
    } catch (const std::exception& e) {
      // Printed as well as reported: a streamed target arms no goal, so for those this line is the
      // only record that the loop died and why.
      std::cerr << "Torque control thread error: " << e.what() << std::endl;
      thread_error = e.what();
    }
    finish_control_thread_(std::move(thread_error));
  }

  // Arm a move and return the id that names it. Every move gets its own id, so an outcome reported by
  // whatever was running before can be told apart from the move now in flight.
  std::uint64_t arm_goal_() {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    goal_status_ = GoalStatus::IN_FLIGHT;
    goal_error_.clear();
    return ++goal_id_;
  }

  // Settle the move `id` names. Two moves are refused: one already settled (a status written over a
  // finished one would report the wrong outcome to a poller), and one that is no longer the move in
  // flight — an older trajectory finishing must not report the arrival of the move that replaced it.
  // Ids start at 1, so 0 names no move and settles nothing.
  //
  // The id is never exposed. It exists for this check: a caller sees one status, and what it needs
  // is for that status to belong to the move it last commanded.
  void settle_goal_(std::uint64_t id, GoalStatus status, std::string reason = {}) {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    if (goal_status_ != GoalStatus::IN_FLIGHT || goal_id_ != id) return;
    goal_status_ = status;
    goal_error_ = std::move(reason);
  }

  // Settle whichever move is in flight, for the paths that end every move there is: a dying control
  // loop and a torn-down one. They cannot name an id — the move they end was armed by someone else.
  void settle_current_goal_(GoalStatus status, std::string reason = {}) {
    std::uint64_t id = 0;
    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      id = goal_id_;
    }
    settle_goal_(id, status, std::move(reason));
  }

  // How the move in flight is doing, by the one criterion both control loops share so a caller reads
  // REACHED the same way whatever mode the arm is in.
  //
  // REACHED requires actual arrival — at the commanded reference and no longer moving. A move that
  // runs out of patience is STALLED, never REACHED: an arm held short by contact has not arrived,
  // and telling a poller it has lets an experiment advance on a robot that is nowhere near its
  // target.
  //
  // Stalling is measured as lost PROGRESS, not low velocity. Under soft gains, or on the last
  // approach to a target, every joint can sit below the rest threshold for a second while the arm is
  // still genuinely closing — judging by velocity alone would reject that legitimate slow move. So
  // the count advances only while the smallest error yet seen for this move stops improving, which
  // an arm held against something does and a converging one does not.
  //
  // Two guards, because that one bounds a move only at `distance / STALL_PROGRESS_EPSILON` seconds:
  // an arm creeping by a hair every second resets the count indefinitely. `move_ticks` is the flat
  // deadline on the move as a whole, and the pair covers both shapes — held fast (1 s) and never
  // quite finishing (60 s).
  enum class Arrival { InFlight, Reached, Stalled };

  static Arrival arrival_(const Vector7d& ref, const Vector7d& q, const Vector7d& dq, double& best_err,
                          int& stall_ticks, int& move_ticks) {
    const double err = (ref - q).cwiseAbs().maxCoeff();
    if (err < SETTLE_POSITION_TOLERANCE && dq.cwiseAbs().maxCoeff() < SETTLE_VELOCITY_TOLERANCE) {
      return Arrival::Reached;
    }
    if (err < best_err - STALL_PROGRESS_EPSILON) {
      best_err = err;
      stall_ticks = 0;
    } else {
      ++stall_ticks;
    }
    return (stall_ticks >= STALL_TICKS_CAP || ++move_ticks >= MOVE_TICKS_CAP) ? Arrival::Stalled
                                                                             : Arrival::InFlight;
  }

  // Apply that outcome to the goal `id` names. Returns true once the move is over, so the caller
  // clears its in-flight id.
  bool settle_on_arrival_(std::uint64_t id, Arrival a) {
    switch (a) {
      case Arrival::Reached:
        complete_goal_(id);
        return true;
      case Arrival::Stalled:
        fail_goal_(id, "the arm did not reach the target: it stopped short, or ran out of time trying");
        return true;
      case Arrival::InFlight:
        return false;
    }
    return false;
  }

  void complete_goal_(std::uint64_t id) { settle_goal_(id, GoalStatus::REACHED); }

  // The move stopped short.
  void fail_goal_(std::uint64_t id, std::string reason) {
    settle_goal_(id, GoalStatus::ABORTED, std::move(reason));
  }

  // Every control-thread exit path must clear control_running_ and then settle the move in flight: a
  // thread that died mid-goal (reflex, NaN, connection loss) otherwise leaves a poller waiting
  // forever on a move nothing can finish — and a goal still pending at exit was aborted, not reached.
  //
  // `reason` is what the loop caught, so the caller learns *why* ("motion aborted by reflex!
  // [cartesian_reflex]") rather than only that something stopped. Nothing else knows: the exception
  // dies with the thread, and by the time the caller looks the robot may already be recovered.
  void finish_control_thread_(std::string reason = {}) {
    control_running_.store(false);
    {
      std::lock_guard<std::mutex> lk(last_state_mutex_);
      last_software_ref_.reset();
    }
    if (reason.empty()) reason = "control loop stopped before the joint target was reached";
    settle_current_goal_(GoalStatus::ABORTED, std::move(reason));
  }

 public:
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
  franka::RobotState read_robot_state_() { return read_state_snapshot_().first; }

  // The cached robot state and the software reference copied under one lock, so a state() built from
  // them pairs q/dq/tau with the q_d of the same control tick.
  std::pair<franka::RobotState, std::optional<Vector7d>> read_state_snapshot_() {
    if (control_running_.load()) {
      std::lock_guard<std::mutex> lk(last_state_mutex_);
      if (last_state_ != nullptr) return {*last_state_, last_software_ref_};
    }
    return {robot_->readOnce(), std::nullopt};
  }

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;
  std::thread control_thread_;
  std::atomic<bool> control_running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> has_target_{false};

  // The move in flight. The goal machinery assumes callers are serialized — the Python bindings hold
  // the GIL for the whole publish — so concurrent C++ callers are not supported.
  std::mutex goal_mutex_;
  GoalStatus goal_status_ = GoalStatus::NONE;
  std::string goal_error_;
  // Names the move `goal_status_` describes. Ids start at 1; 0 names no move.
  std::uint64_t goal_id_ = 0;

  std::mutex last_state_mutex_;
  std::unique_ptr<franka::RobotState> last_state_;
  // The software-impedance loop's reference, published so state() can report the tracked q_d; empty
  // whenever the torque loop is not running.
  std::optional<Vector7d> last_software_ref_;

  const double relative_dynamics_factor_{1.0};
  // Read when a motion command starts the loop; the running loop never reads it — a software gains
  // change reaches the loop via new_gains_. Callers are serialized by the Python GIL.
  ControlMode control_mode_;
  std::mutex target_mutex_;
  Vector7d target_q_ = Vector7d::Zero();
  // The move `target_q_` belongs to, carried with it so the loop learns which goal it is executing
  // from the same critical section that hands it the target; 0 for a streamed target, which has none.
  std::uint64_t target_goal_id_ = 0;
  // A gains-only SoftwareImpedance change, picked up by the running torque loop mid-session.
  SoftwareImpedance new_gains_;
  std::atomic<bool> has_new_gains_{false};

  // The unconditional half of set_control_mode; the constructor uses it to force-apply the initial mode.
  void apply_control_mode_(const ControlMode& mode) {
    // A software→software change is absorbed by the running torque loop (the rate limiter and lowpass
    // smooth the gain step, as with reference steps); every other change needs the backend torn down.
    const bool gains_handoff = std::holds_alternative<SoftwareImpedance>(mode) &&
                               std::holds_alternative<SoftwareImpedance>(control_mode_);
    if (!gains_handoff) stop_control_loop_();
    control_mode_ = mode;
    if (const auto* internal = std::get_if<InternalImpedance>(&control_mode_)) {
      robot_->setJointImpedance(internal->k_theta);
    } else {
      std::lock_guard<std::mutex> lk(target_mutex_);
      new_gains_ = std::get<SoftwareImpedance>(control_mode_);
      has_new_gains_.store(true);
    }
  }

  void stop_control_loop_() {
    stop_requested_.store(true);
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
    control_running_.store(false);
    has_target_.store(false);
    settle_current_goal_(GoalStatus::REACHED);
    stop_requested_.store(false);
  }
};

}  // namespace positronic_franka
