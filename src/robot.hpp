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

#include "control_loop_watch.hpp"
#include "impedance_law.hpp"
#include "pose_ik.hpp"
#include "trajectory_generator.hpp"

namespace positronic_franka {

// Arrival: joints within tolerance of the reference and no longer moving, judged on one reading.
//
// KNOWN LIMITATION, accepted: an arm whose resting point is offset from the target by load or
// friction dips into the band at near-zero speed at one end of its swing and settles outside it, so
// that single reading reports REACHED. The status is only a report — the loop goes on driving the
// target until a new one arrives — but it is never taken back, so a caller polling at that instant
// acts on it. Fix, if it ever bites: let the caller pass the tolerances as it passes the deadline.
// internal#162.
constexpr double SETTLE_POSITION_TOLERANCE = 0.05;  // rad
constexpr double SETTLE_VELOCITY_TOLERANCE = 0.05;  // rad/s
// The stop-and-settle phase gives up after this many ticks (1 kHz), so an arm that never goes quiet
// still releases the control loop.
constexpr int SETTLE_TICKS_CAP = 1000;
// Deadline used when the caller names none. Generous for a joint move on this arm.
constexpr double DEFAULT_MOVE_DEADLINE_S = 15.0;
// A running control loop with no robot state for this long counts as stuck in libfranka. Far above any
// normal gap: at the end of a motion, libfranka calls no callback for up to about 1.2 s.
constexpr double DEFAULT_CONTROL_LINK_TIMEOUT_S = 10.0;


// Control modes. InternalImpedance drives the robot's built-in joint impedance controller through the
// joint position motion generator with Ruckig-shaped references. SoftwareImpedance runs the impedance
// law here instead, over the torque interface, moving the reference to each target at a capped joint speed.
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

// Per-joint spring torque rate cap, Nm/s, tuned for the default SoftwareImpedance gains.
constexpr std::array<double, 7> DEFAULT_MAX_SPRING_TORQUE_RATE = {130.0, 130.0, 130.0, 130.0, 85.0, 60.0, 35.0};
// Per-joint speed, rad/s, at which the spring reference q_d moves to a new target. Recorded policy motion stays
// below 1.3 rad/s on every joint over any 0.2 s. The FR3 limit on joints 1 to 4 is 2.62 rad/s.
constexpr std::array<double, 7> DEFAULT_MAX_REFERENCE_SPEED = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
// The point the SoftwareImpedance Cartesian terms act at, in flange axes from the flange origin, m: the flange.
constexpr std::array<double, 3> DEFAULT_STIFFNESS_POINT = {0.0, 0.0, 0.0};

// Gain defaults are copied from fairo polymetis/conf/robot_client/franka_hardware.yaml, the
// configuration HybridJointImpedanceControl runs with in the DROID data-collection stack.
struct SoftwareImpedance {
  std::array<double, 7> kq{40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0};
  std::array<double, 7> kqd{4.0, 6.0, 5.0, 5.0, 3.0, 2.0, 1.0};
  std::array<double, 6> kx{750.0, 750.0, 750.0, 15.0, 15.0, 15.0};
  std::array<double, 6> kxd{37.0, 37.0, 37.0, 2.0, 2.0, 2.0};
  // Per-joint cap on the rate of change of the spring torque, Nm/s.
  std::array<double, 7> max_spring_torque_rate = DEFAULT_MAX_SPRING_TORQUE_RATE;
  // Per-joint speed at which the spring reference q_d moves to a new target, rad/s.
  std::array<double, 7> max_reference_speed = DEFAULT_MAX_REFERENCE_SPEED;
  // The point the Cartesian terms act at, in flange axes from the flange origin, m. The default gains are tuned at
  // the flange.
  std::array<double, 3> stiffness_point = DEFAULT_STIFFNESS_POINT;

  SoftwareImpedance() = default;
  explicit SoftwareImpedance(const std::array<double, 7>& max_spring_torque_rate_in,
                             const std::array<double, 7>& max_reference_speed_in = DEFAULT_MAX_REFERENCE_SPEED,
                             const std::array<double, 3>& stiffness_point_in = DEFAULT_STIFFNESS_POINT)
      : max_spring_torque_rate(max_spring_torque_rate_in),
        max_reference_speed(max_reference_speed_in),
        stiffness_point(stiffness_point_in) {
    validate_guards_();
  }
  SoftwareImpedance(const std::array<double, 7>& kq_in, const std::array<double, 7>& kqd_in,
                    const std::array<double, 6>& kx_in, const std::array<double, 6>& kxd_in,
                    const std::array<double, 7>& max_spring_torque_rate_in = DEFAULT_MAX_SPRING_TORQUE_RATE,
                    const std::array<double, 7>& max_reference_speed_in = DEFAULT_MAX_REFERENCE_SPEED,
                    const std::array<double, 3>& stiffness_point_in = DEFAULT_STIFFNESS_POINT)
      : kq(kq_in),
        kqd(kqd_in),
        kx(kx_in),
        kxd(kxd_in),
        max_spring_torque_rate(max_spring_torque_rate_in),
        max_reference_speed(max_reference_speed_in),
        stiffness_point(stiffness_point_in) {
    validate_half(kq, kqd, "joint (kq/kqd)");
    validate_half(kx, kxd, "Cartesian (kx/kxd)");
    if (all_zero(kq) && all_zero(kx)) {
      throw std::invalid_argument("at least one of the joint or Cartesian halves must be active");
    }
    validate_guards_();
  }

 private:
  void validate_guards_() const {
    if (!all_positive(max_spring_torque_rate)) {
      throw std::invalid_argument("max_spring_torque_rate must be finite and strictly positive");
    }
    if (!all_positive(max_reference_speed)) {
      throw std::invalid_argument("max_reference_speed must be finite and strictly positive");
    }
    if (!std::all_of(stiffness_point.begin(), stiffness_point.end(), [](double v) { return std::isfinite(v); })) {
      throw std::invalid_argument("stiffness_point must be finite");
    }
  }
};

inline bool operator==(const InternalImpedance& a, const InternalImpedance& b) {
  return a.k_theta == b.k_theta;
}

inline bool operator==(const SoftwareImpedance& a, const SoftwareImpedance& b) {
  return a.kq == b.kq && a.kqd == b.kqd && a.kx == b.kx && a.kxd == b.kxd &&
         a.max_spring_torque_rate == b.max_spring_torque_rate && a.max_reference_speed == b.max_reference_speed &&
         a.stiffness_point == b.stiffness_point;
}

using ControlMode = std::variant<InternalImpedance, SoftwareImpedance>;

// Robot construction defaults, shared with the Python bindings.
constexpr franka::RealtimeConfig DEFAULT_REALTIME_CONFIG = franka::RealtimeConfig::kIgnore;
constexpr double DEFAULT_RELATIVE_DYNAMICS_FACTOR = 1.0;
inline const ControlMode DEFAULT_CONTROL_MODE = InternalImpedance{};

// How the joint move now in flight is going. Every `set_target_joints` starts one; a newer target
// replaces an older one, and only the move in flight has a status — the replaced one is never
// reported. Polled, never blocking: a caller that blocks inside the library stops running the loop
// that clears robot errors, so a reflex mid-move goes uncleared and the move never ends.
enum class GoalStatus {
  // No move has been commanded yet on this Robot.
  NONE,
  // Commanded, not finished.
  IN_FLIGHT,
  // The arm settled at the commanded target.
  REACHED,
  // The move stopped short — a reflex, a rejected plan, a lost connection, an expired deadline.
  // `Goal::reason` says which. The arm may still be braking when this appears; read `state()` if
  // that matters.
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
  // Flange pose in base (robot) frame, O_T_F: [tx, ty, tz, qw, qx, qy, qz]
  Vector7d flange_pose;
  // Robot controller time since start, seconds.
  double time = 0.0;
  int error = 0;
  std::string error_message;
  // External wrench (force, torque) on end-effector frame expressed in K frame.
  Vector6d end_effector_wrench = Vector6d::Zero();
};

class Robot {
 public:
  explicit Robot(const std::string& ip,
                 franka::RealtimeConfig realtime_config = DEFAULT_REALTIME_CONFIG,
                 double relative_dynamics_factor = DEFAULT_RELATIVE_DYNAMICS_FACTOR,
                 ControlMode control_mode = DEFAULT_CONTROL_MODE,
                 double control_link_timeout_s = DEFAULT_CONTROL_LINK_TIMEOUT_S)
      : loop_watch_(control_link_timeout_(control_link_timeout_s)),
        control_link_lost_(control_link_lost_message_(control_link_timeout_s)),
        robot_(std::make_unique<franka::Robot>(ip, realtime_config)),
        relative_dynamics_factor_(std::clamp(relative_dynamics_factor, 0.0001, 1.0)) {
    model_ = std::make_unique<franka::Model>(robot_->loadModel());
    // Force-apply: the equality guard in set_control_mode must not skip pushing k_theta when the
    // requested mode happens to equal the default — the robot may persist stiffness from past sessions.
    apply_control_mode_(control_mode);
  }

  ~Robot() {
    try {
      stop_control_loop_();
    } catch (const std::exception& e) {
      // A stuck loop still uses this object, so it cannot be left behind.
      std::cerr << "Robot teardown: " << e.what() << "; waiting for the control loop" << std::endl;
      control_thread_.join();
    }
  }

  State state() {
    auto [rs, software_ref] = read_state_snapshot_();
    const Eigen::Matrix4d T = flange_pose(Eigen::Map<const Eigen::Matrix4d>(rs.O_T_EE.data()),
                                          Eigen::Map<const Eigen::Matrix4d>(rs.F_T_EE.data()));
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
    st.flange_pose << t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z();
    st.time = rs.time.toSec();
    st.error = rs.current_errors ? 1 : 0;
    if (st.error) {
      st.error_message = static_cast<std::string>(rs.current_errors);
    }
    // NOTE: This relies on the fact that we don't configure EE_T_K frame.
    st.end_effector_wrench = Eigen::Map<const Vector6d>(rs.K_F_ext_hat_K.data());
    // A stuck loop serves the same cached state for good, so no robot error can appear in it.
    if (end_stuck_loop_()) {
      st.error = 1;
      st.error_message += st.error_message.empty() ? control_link_lost_ : "; " + control_link_lost_;
    }
    return st;
  }

  // Apply a mode with the least interruption it allows: an equal mode is a no-op; a gains-only
  // SoftwareImpedance change reaches the running torque loop without stopping motion; anything else
  // stops the loop (clearing the pending target) and the next command starts the matching one.
  // k_theta is pushed to the robot; software gains live only in the torque loop.
  void set_control_mode(const ControlMode& mode) {
    if (mode == control_mode_) return;
    apply_control_mode_(mode);
  }

  ControlMode control_mode() const { return control_mode_; }

  // The move in flight, read without blocking.
  Goal goal() {
    // A stuck loop never reaches its deadline check, so end its move here.
    end_stuck_loop_();
    std::lock_guard<std::mutex> lk(goal_mutex_);
    return Goal{goal_status_, goal_error_};
  }

  // Command a joint move and return immediately. Poll `goal()` for IN_FLIGHT / REACHED / ABORTED.
  //
  // Starts the control loop if it is not running, then publishes the target to it. Both control
  // modes report the same way; how the arm gets there is the mode's business.
  //
  // `deadline_s` bounds the move: it ABORTS if the arm has not settled by then. An argument, not a
  // constant, because reach, gains and payload decide what is reasonable and only the caller knows
  // them.
  void set_target_joints(const Eigen::Ref<const Vector7d>& q_target,
                         double deadline_s = DEFAULT_MOVE_DEADLINE_S) {
    // A torque-mode target becomes the reference verbatim, so a NaN here would become NaN torques.
    if (!q_target.allFinite()) {
      throw std::invalid_argument("q_target must be finite");
    }
    // NaN fails every comparison and infinity never arrives, so either would leave a stuck move
    // IN_FLIGHT for good; zero or negative would abort on the first callback.
    if (!std::isfinite(deadline_s) || deadline_s <= 0.0) {
      throw std::invalid_argument("deadline_s must be finite and strictly positive");
    }
    std::uint64_t goal_id = 0;
    if (!control_running_.load()) {
      if (control_thread_.joinable()) {
        control_thread_.join();
      }
      // Arm between the join and the launch, so exactly one loop can settle this goal. Later, and a
      // loop dying before the goal exists leaves it armed on a dead thread, IN_FLIGHT forever.
      // Earlier, and the outgoing loop can abort the move its successor is about to run.
      goal_id = arm_goal_();
      // Starting a loop issues a Move command, which the robot refuses while it holds an error
      // ("command not possible in the current mode"), so the fresh thread would die on its first
      // tick. Report the robot's own error instead. Clearing is the caller's call: a reflex means
      // the arm hit something, and only the caller knows whether resuming is safe.
      // Best effort — if the state read itself fails, let the loop report what is wrong.
      std::string robot_errors;
      std::unique_ptr<franka::RobotState> preflight;
      try {
        preflight = std::make_unique<franka::RobotState>(read_robot_state_());
        if (const auto errors = preflight->current_errors; static_cast<bool>(errors)) {
          robot_errors = static_cast<std::string>(errors);
        }
      } catch (const std::exception&) {  // NOLINT(bugprone-empty-catch)
      }
      if (!robot_errors.empty()) {
        fail_goal_(goal_id, "robot reports an error, recover before commanding a move: " + robot_errors);
        return;
      }
      stop_requested_.store(false);
      // Drop anything a dead loop left queued — no loop consumed it, and this call's own publish
      // comes below, so the replacement loop's first ticks must not see the stale target.
      has_target_.store(false);
      {
        std::lock_guard<std::mutex> lk(target_mutex_);
        target_goal_id_ = 0;
      }
      // Seed the cache from the read above, so state() has something to serve the moment the loop
      // counts as running. libfranka allows one operation on the connection at a time, so a state()
      // that falls through to readOnce() in the gap before the loop's first tick takes the
      // connection the control thread is claiming, and one of the two dies.
      {
        std::lock_guard<std::mutex> lk(last_state_mutex_);
        last_state_ = std::move(preflight);
        last_software_ref_.reset();
      }
      loop_watch_.start();
      control_running_.store(true);
      // Snapshot the mode here: the caller is GIL-serialized with set_control_mode, while the thread
      // body would race a concurrent gains handoff writing control_mode_.
      const ControlMode mode = control_mode_;
      try {
        control_thread_ = std::thread([this, mode] {
          if (const auto* software = std::get_if<SoftwareImpedance>(&mode)) {
            this->run_joint_torque_control_(*software);
          } else {
            this->run_joint_position_control_(std::get<InternalImpedance>(mode));
          }
        });
      } catch (...) {
        // No thread means nothing will ever clear control_running_ or settle the goal armed above.
        control_running_.store(false);
        loop_watch_.exited();
        fail_goal_(goal_id, "could not start the control loop");
        throw;
      }
    } else {
      // Loop already running (every move after the first): no join to order against, so arming
      // immediately before the publish below is enough. If it dies in the window,
      // `finish_control_thread_` aborts this goal with the loop's own reason.
      goal_id = arm_goal_();
    }
    {
      std::lock_guard<std::mutex> lk(target_mutex_);
      target_q_ = q_target;
      target_goal_id_ = goal_id;
      // Carried with the target, so a later call cannot retroactively change this move's deadline.
      target_deadline_s_ = deadline_s;
      has_target_.store(true);
    }
    // The loop can exit between the `control_running_` read above and this publish, leaving the goal
    // IN_FLIGHT with nothing left to settle it — so re-read and abort it here. A false read is final
    // (callers are serialized), and `settle_goal_` ignores an already-settled goal, so a
    // `finish_control_thread_` landing in between wins with its own, better reason.
    if (!control_running_.load()) {
      fail_goal_(goal_id, "the control loop stopped before it took up the target");
    }
  }

  // Forward Kinematics: compute the flange pose (tx, ty, tz, qw, qx, qy, qz) from joints q (7,)
  Vector7d forward_kinematics(
      const Eigen::Ref<const Vector7d>& q) {
    franka::RobotState st = read_robot_state_();
    st.q = to_std_array7_(q);

    const Eigen::Matrix4d T = flange_pose_matrix_(st);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Quaterniond quat(R);

    Vector7d pose;
    pose << t.x(), t.y(), t.z(), quat.w(), quat.x(), quat.y(), quat.z();
    return pose;
  }

  // Model terms at explicit joint values, with the robot's current frames and load. The same terms
  // the SoftwareImpedance loop uses, so logged traces can be checked offline against the law. The
  // Jacobian is at `stiffness_point`, in flange axes from the flange origin, where the law's Cartesian gains act.
  SpatialJacobian zero_jacobian(const Eigen::Ref<const Vector7d>& q, const std::array<double, 3>& stiffness_point) {
    franka::RobotState st = read_robot_state_();
    st.q = to_std_array7_(q);
    return flange_point_jacobian_(st, Eigen::Map<const Eigen::Vector3d>(stiffness_point.data()));
  }

  Vector7d coriolis(const Eigen::Ref<const Vector7d>& q, const Eigen::Ref<const Vector7d>& dq) {
    franka::RobotState st = read_robot_state_();
    st.q = to_std_array7_(q);
    st.dq = to_std_array7_(dq);
    const auto cor = model_->coriolis(st);
    return Eigen::Map<const Vector7d>(cor.data());
  }

  // Inverse Kinematics to the flange pose in base frame (tx, ty, tz, qw, qx, qy, qz)
  Vector7d inverse_kinematics(
      const Eigen::Ref<const Vector7d>& flange_pose_wxyz,
      double tol = 1e-4, int max_iters = 150, double min_step = 1e-8, double pinv_reg = 0.03,
      double nullspace_gain = 0.002, double line_search_alpha = 1.0, double line_search_beta = 0.5,
      int line_search_max_steps = 20) {
    auto base = read_robot_state_();
    Vector7d q0 = Eigen::Map<const Vector7d>(base.q.data());
    return inverse_kinematics_q0(flange_pose_wxyz, q0, tol, max_iters, min_step, pinv_reg, nullspace_gain,
                                 line_search_alpha, line_search_beta, line_search_max_steps);
  }

  Vector7d inverse_kinematics_q0(
      const Eigen::Ref<const Vector7d>& flange_pose_wxyz,
      const Eigen::Ref<const Vector7d>& q0,
      double tol = 1e-4, int max_iters = 150, double min_step = 1e-8, double pinv_reg = 0.03,
      double nullspace_gain = 0.002, double line_search_alpha = 1.0, double line_search_beta = 0.5,
      int line_search_max_steps = 20) {
    franka::RobotState st = read_robot_state_();
    const auto kinematics = [&](const Vector7d& q) {
      st.q = to_std_array7_(q);
      return ToolKinematics{flange_pose_matrix_(st), flange_jacobian_(st)};
    };
    return solve_damped_pose_ik(kinematics, flange_pose_wxyz, q0, tol, max_iters, min_step, pinv_reg, nullspace_gain,
                                line_search_alpha, line_search_beta, line_search_max_steps);
  }

  // The IK for a live command: the joints that bring the flange as close as the bounds allow to
  // flange_pose_wxyz, from the measured joints.
  Vector7d inverse_kinematics_with_limits(const Eigen::Ref<const Vector7d>& flange_pose_wxyz) {
    const franka::RobotState base = read_robot_state_();
    return inverse_kinematics_with_limits(flange_pose_wxyz, Eigen::Map<const Vector7d>(base.q.data()),
                                          IK_MAX_JOINT_STEP);
  }

  Vector7d inverse_kinematics_with_limits(const Eigen::Ref<const Vector7d>& flange_pose_wxyz,
                                          const Eigen::Ref<const Vector7d>& q0, double max_step) {
    franka::RobotState st = read_robot_state_();
    const auto kinematics = [&](const Vector7d& q) {
      st.q = to_std_array7_(q);
      return ToolKinematics{flange_pose_matrix_(st), flange_jacobian_(st)};
    };
    return solve_pose_ik(kinematics, flange_pose_wxyz, q0, max_step);
  }

private:

  // Utilities for IK readability
  static std::array<double, 7> to_std_array7_(const Vector7d& v) {
    std::array<double, 7> a{};
    for (size_t i = 0; i < 7; ++i) a[i] = v(i);
    return a;
  }

  Eigen::Matrix4d flange_pose_matrix_(const franka::RobotState& st) const {
    const auto T_data = model_->pose(franka::Frame::kFlange, st);
    return Eigen::Map<const Eigen::Matrix4d>(T_data.data());
  }

  SpatialJacobian flange_jacobian_(const franka::RobotState& st) const {
    const auto J_data = model_->zeroJacobian(franka::Frame::kFlange, st);
    return Eigen::Map<const SpatialJacobian>(J_data.data());
  }

  SpatialJacobian flange_point_jacobian_(const franka::RobotState& st, const Eigen::Vector3d& flange_to_point) const {
    return point_jacobian(ToolKinematics{flange_pose_matrix_(st), flange_jacobian_(st)}, flange_to_point);
  }

 public:
  double relative_dynamics_factor() const { return relative_dynamics_factor_; }

 private:
  void run_joint_position_control_(InternalImpedance) {
    std::string thread_error;
    try {
      TrajectoryGenerator traj(relative_dynamics_factor_);

      robot_->control(
        [&, this,
         first = true,
         goal_in_flight = std::uint64_t{0},
         elapsed_s = 0.0,
         deadline_s = 0.0,
         stopping = false](const franka::RobotState& st, franka::Duration period) mutable -> franka::JointPositions {
          {
            std::lock_guard<std::mutex> lk(last_state_mutex_);
            last_state_ = std::make_unique<franka::RobotState>(st);
          }
          loop_watch_.tick();

          if (first) {
            traj.start_at_rest(st.q_d);
            first = false;
          } else if (!stopping && stop_requested_.load()) {
            stopping = true;
            has_target_.store(false);
            traj.brake_to_rest();
          }

          if (!stopping && has_target_.load()) {
            std::lock_guard<std::mutex> lk(target_mutex_);
            const bool planned = traj.set_target(target_q_);
            has_target_.store(false);
            goal_in_flight = std::exchange(target_goal_id_, 0);
            elapsed_s = 0.0;
            // Plan duration plus the settle budget, so a slow `relative_dynamics_factor` gets the
            // time it asked for while a move that overruns its own plan still ends.
            deadline_s = std::exchange(target_deadline_s_, 0.0);
            // A rejected plan leaves the previous trajectory playing, so stop it: the goal is about
            // to report ABORTED, and a caller acting on that must not have the arm still driving
            // toward the superseded target.
            if (!planned) {
              fail_goal_(goal_in_flight, "joint target rejected by the trajectory planner");
              goal_in_flight = 0;
              traj.brake_to_rest();
            }
          }

          auto pos = traj.step(period.toSec());

          // Arrival is physical — at the reference and not moving — not an exhausted trajectory,
          // which the internal controller only tracks approximately. `traj.active()` is passed
          // rather than gating the call: there is no arrival to test while the reference is still
          // being shaped (a Ruckig move starts with both at rest, which would read as REACHED on
          // tick one), but the deadline must keep running through that phase.
          if (goal_in_flight != 0) {
            const Arrival outcome = arrival_(Eigen::Map<const Vector7d>(pos.data()),
                                             Eigen::Map<const Vector7d>(st.q.data()),
                                             Eigen::Map<const Vector7d>(st.dq.data()), traj.active(),
                                             period.toSec(), deadline_s, elapsed_s);
            if (settle_on_arrival_(goal_in_flight, outcome)) {
              goal_in_flight = 0;
              // ABORTED says the move stopped short, so the arm has to actually stop rather than
              // keep being driven at the expired target.
              if (outcome == Arrival::Stalled) traj.brake_to_rest();
            }
          }

          if (!traj.active() && stopping) {
            auto cmd = franka::JointPositions(pos);
            cmd.motion_finished = true;
            return cmd;
          }

          return franka::JointPositions(pos);
        });
    } catch (const std::exception& e) {
      // Printed as well as reported: a streamed target arms no goal, so this is its only record.
      std::cerr << "Joint control thread error: " << e.what() << std::endl;
      thread_error = e.what();
    }
    finish_control_thread_(std::move(thread_error));
  }

  // Torque-interface control loop running the polymetis hybrid impedance law:
  //   tau = (J^T Kx J + Kq)(q_d - q) - (J^T Kxd J + Kqd) dq + coriolis
  // with the spring term rate-capped (impedance_law.hpp). Gravity is compensated by libfranka underneath the
  // torque command. Shares the target/stop/goal machinery with the position loop; only the reference differs:
  // q_d moves to each target in a straight line at max_reference_speed, where the position loop shapes a Ruckig
  // trajectory.
  void run_joint_torque_control_(SoftwareImpedance imp) {
    Vector7d Kq = Eigen::Map<const Vector7d>(imp.kq.data());
    Vector7d Kqd = Eigen::Map<const Vector7d>(imp.kqd.data());
    Vector6d Kx = Eigen::Map<const Vector6d>(imp.kx.data());
    Vector6d Kxd = Eigen::Map<const Vector6d>(imp.kxd.data());
    SpringTorqueRateLimiter spring_limiter(imp.max_spring_torque_rate);
    Vector7d max_reference_speed = Eigen::Map<const Vector7d>(imp.max_reference_speed.data());
    Eigen::Vector3d stiffness_point = Eigen::Map<const Eigen::Vector3d>(imp.stiffness_point.data());
    std::string thread_error;
    try {
      robot_->control(
        [&, this,
         first = true,
         goal_in_flight = std::uint64_t{0},
         elapsed_s = 0.0,
         deadline_s = 0.0,
         stopping = false,
         stop_ticks = 0,
         ref = Vector7d(Vector7d::Zero()),
         target = Vector7d(Vector7d::Zero())](const franka::RobotState& st,
                                              franka::Duration period) mutable -> franka::Torques {
          const Vector7d q = Eigen::Map<const Vector7d>(st.q.data());
          const Vector7d dq = Eigen::Map<const Vector7d>(st.dq.data());

          if (first) {
            ref = target = q;
            first = false;
          } else if (!stopping && stop_requested_.load()) {
            stopping = true;
            has_target_.store(false);
            ref = target = q;
          }

          if (!stopping && has_target_.load()) {
            std::lock_guard<std::mutex> lk(target_mutex_);
            target = target_q_;
            goal_in_flight = std::exchange(target_goal_id_, 0);
            elapsed_s = 0.0;
            deadline_s = std::exchange(target_deadline_s_, 0.0);
            has_target_.store(false);
          }

          if (has_new_gains_.load()) {
            std::lock_guard<std::mutex> lk(target_mutex_);
            Kq = Eigen::Map<const Vector7d>(new_gains_.kq.data());
            Kqd = Eigen::Map<const Vector7d>(new_gains_.kqd.data());
            Kx = Eigen::Map<const Vector6d>(new_gains_.kx.data());
            Kxd = Eigen::Map<const Vector6d>(new_gains_.kxd.data());
            spring_limiter.set_max_rate(new_gains_.max_spring_torque_rate);
            max_reference_speed = Eigen::Map<const Vector7d>(new_gains_.max_reference_speed.data());
            stiffness_point = Eigen::Map<const Eigen::Vector3d>(new_gains_.stiffness_point.data());
            has_new_gains_.store(false);
          }

          // No trajectory here: the impedance law moves the arm. The speed cap on q_d keeps a large jump in the
          // target from becoming a large spring pull, at any command rate.
          ref = move_reference_toward(ref, target, max_reference_speed, period.toSec());

          // The position loop's arrival rule: there is no arrival to test while the reference still moves.
          if (goal_in_flight != 0) {
            const Arrival outcome =
                arrival_(ref, q, dq, /*travelling=*/ref != target, period.toSec(), deadline_s, elapsed_s);
            if (settle_on_arrival_(goal_in_flight, outcome)) {
              goal_in_flight = 0;
              // As above: hold position rather than keep pulling toward a target already reported
              // as not reached.
              if (outcome == Arrival::Stalled) ref = target = q;
            }
          }

          // Publish the state and this tick's final reference in one critical section, so a state()
          // snapshot can never pair them from different ticks.
          {
            std::lock_guard<std::mutex> lk(last_state_mutex_);
            last_state_ = std::make_unique<franka::RobotState>(st);
            last_software_ref_ = ref;
          }
          loop_watch_.tick();

          const SpatialJacobian J = flange_point_jacobian_(st, stiffness_point);
          const auto cor = model_->coriolis(st);

          const Matrix7d Kp = J.transpose() * Kx.asDiagonal() * J + Matrix7d(Kq.asDiagonal());
          const Matrix7d Kd = J.transpose() * Kxd.asDiagonal() * J + Matrix7d(Kqd.asDiagonal());
          const Vector7d tau = impedance_torque(Kp, Kd, ref, q, dq, Eigen::Map<const Vector7d>(cor.data()),
                                                spring_limiter, period.toSec());

          std::array<double, 7> tau_arr;
          Eigen::Map<Vector7d>(tau_arr.data()) = tau;
          franka::Torques cmd(tau_arr);

          if (stopping) {
            // The spring at ref = q plus damping brings the arm to rest; finish once settled (capped).
            ++stop_ticks;
            if (stop_has_settled(spring_limiter, dq, SETTLE_VELOCITY_TOLERANCE) || stop_ticks >= SETTLE_TICKS_CAP) {
              cmd.motion_finished = true;
            }
          }
          return cmd;
        },
        true /* limit_rate */, 100.0 /* cutoff_frequency */);
    } catch (const std::exception& e) {
      // Printed as well as reported: a streamed target arms no goal, so this is its only record.
      std::cerr << "Torque control thread error: " << e.what() << std::endl;
      thread_error = e.what();
    }
    finish_control_thread_(std::move(thread_error));
  }

  // Arm a move and return the id that names it, so an outcome from whatever ran before can be told
  // apart from the move now in flight.
  std::uint64_t arm_goal_() {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    goal_status_ = GoalStatus::IN_FLIGHT;
    goal_error_.clear();
    return ++goal_id_;
  }

  // Settle the move `id` names. Refuses one already settled, and one that is no longer in flight —
  // an older trajectory finishing must not report the arrival of the move that replaced it. Ids
  // start at 1, so 0 names no move. Never exposed; it exists only for this check.
  void settle_goal_(std::uint64_t id, GoalStatus status, std::string reason = {}) {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    if (goal_status_ != GoalStatus::IN_FLIGHT || goal_id_ != id) return;
    goal_status_ = status;
    goal_error_ = std::move(reason);
  }

  // Settle whichever move is in flight, for the paths that cannot name an id: a dying control loop
  // and a torn-down one.
  void settle_current_goal_(GoalStatus status, std::string reason = {}) {
    std::uint64_t id = 0;
    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      id = goal_id_;
    }
    settle_goal_(id, status, std::move(reason));
  }

  enum class Arrival { InFlight, Reached, Stalled };

  // The one criterion both control loops share, so REACHED means the same thing in either mode:
  // every joint inside the tolerance band and no longer moving. Stalled is the caller's deadline;
  // nothing here infers a timescale from the arm. An arm held short by contact is stalled.
  static Arrival arrival_(const Vector7d& ref, const Vector7d& q, const Vector7d& dq, bool travelling,
                          double dt, double deadline_s, double& elapsed_s) {
    // Seconds accumulated from the callback's own period, not a tick count: a callback is only
    // nominally 1 ms, so counting invocations under-measures real time under bad scheduling.
    elapsed_s += dt;
    // Arrival is tested first, so an arm that is at the target on the tick its deadline expires
    // reports REACHED. The deadline exists to end a move that is not arriving; one that has
    // arrived is not that move. While the reference is still being shaped there is nothing to
    // test, and only the deadline runs.
    if (!travelling && (ref - q).cwiseAbs().maxCoeff() < SETTLE_POSITION_TOLERANCE &&
        dq.cwiseAbs().maxCoeff() < SETTLE_VELOCITY_TOLERANCE) {
      return Arrival::Reached;
    }
    return elapsed_s >= deadline_s ? Arrival::Stalled : Arrival::InFlight;
  }

  // Apply that outcome to the goal `id` names. Returns true once the move is over, so the caller
  // clears its in-flight id.
  bool settle_on_arrival_(std::uint64_t id, Arrival a) {
    switch (a) {
      case Arrival::Reached:
        complete_goal_(id);
        return true;
      case Arrival::Stalled:
        fail_goal_(id, "the arm did not reach the target within the deadline");
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

  // Every control-thread exit path clears control_running_ and settles the move in flight: a thread
  // that died mid-goal (reflex, NaN, connection loss) would otherwise leave a poller waiting forever,
  // and a goal still pending at exit was aborted, not reached. `reason` is what the loop caught
  // ("motion aborted by reflex! [cartesian_reflex]") — nothing else knows it, the exception dies
  // with the thread.
  void finish_control_thread_(std::string reason = {}) {
    control_running_.store(false);
    {
      std::lock_guard<std::mutex> lk(last_state_mutex_);
      last_software_ref_.reset();
    }
    if (reason.empty()) reason = "control loop stopped before the joint target was reached";
    settle_current_goal_(GoalStatus::ABORTED, std::move(reason));
    loop_watch_.exited();
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

    // The base URDF stops at the flange (link8), where the published pose and the IK are. Append the
    // end-effector frame F_T_EE that Desk sets for the mounted tool.
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
            << "       franka::RobotState at the time of this call: the tool frame that\n"
            << "       Desk sets. The driver's pose and IK are at the flange. -->\n"
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

  // Returns whether the running loop is stuck. A stuck loop's move is aborted, so the loop must brake
  // if it ever ticks again: the caller acts as if the move ended.
  bool end_stuck_loop_() {
    if (!loop_watch_.stopped_ticking()) return false;
    stop_requested_.store(true);
    has_target_.store(false);
    settle_current_goal_(GoalStatus::ABORTED, control_link_lost_);
    return true;
  }

  static ControlLoopWatch::Clock::duration control_link_timeout_(double seconds) {
    if (!std::isfinite(seconds) || seconds <= 0.0) {
      throw std::invalid_argument("control_link_timeout_s must be finite and strictly positive");
    }
    return std::chrono::duration_cast<ControlLoopWatch::Clock::duration>(std::chrono::duration<double>(seconds));
  }

  static std::string control_link_lost_message_(double seconds) {
    std::ostringstream m;
    m << "no robot state for " << seconds << " s: the control loop is stuck in libfranka and the link to the robot is lost";
    return m.str();
  }

  // The cached robot state and the software reference copied under one lock, so a state() built from
  // them pairs q/dq/tau with the q_d of the same control tick.
  std::pair<franka::RobotState, std::optional<Vector7d>> read_state_snapshot_() {
    if (control_running_.load()) {
      std::lock_guard<std::mutex> lk(last_state_mutex_);
      if (last_state_ != nullptr) return {*last_state_, last_software_ref_};
    }
    return {robot_->readOnce(), std::nullopt};
  }

  // Declared before robot_, so a bad timeout raises before the connection opens.
  ControlLoopWatch loop_watch_;
  const std::string control_link_lost_;
  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;
  std::thread control_thread_;
  std::atomic<bool> control_running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> has_target_{false};

  // The move in flight. Assumes serialized callers (the bindings hold the GIL for the whole
  // publish); concurrent C++ callers are not supported.
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
  // The move `target_q_` belongs to, handed to the loop in the same critical section as the target;
  // 0 for a streamed target, which has none.
  std::uint64_t target_goal_id_ = 0;
  // The deadline the pending target was commanded with, in seconds; taken by the loop with it.
  double target_deadline_s_ = 0.0;
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

  // Raises when the loop is stuck in libfranka: nothing can end that call, so the thread stays, and
  // stop_requested_ stays set so that the loop stops if it ever ticks again.
  void stop_control_loop_() {
    stop_requested_.store(true);
    if (control_thread_.joinable()) {
      if (loop_watch_.wait_for_exit() == LoopEnd::STOPPED_TICKING) {
        settle_current_goal_(GoalStatus::ABORTED, control_link_lost_);
        throw std::runtime_error("cannot stop the control loop: " + control_link_lost_);
      }
      control_thread_.join();
    }
    control_running_.store(false);
    has_target_.store(false);
    // A teardown ends the move wherever the arm got to, so ABORTED. Normally the loop's own
    // `finish_control_thread_` has already settled it with a better reason and this is ignored;
    // it stands for the paths where nothing else settles the goal.
    settle_current_goal_(GoalStatus::ABORTED, "the control loop was stopped before the joint target was reached");
    stop_requested_.store(false);
  }
};

}  // namespace positronic_franka
