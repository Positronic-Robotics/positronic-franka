#pragma once

// A stand-in for the part of libfranka that src/robot.hpp uses. The control call ticks at 1 ms on a
// joint model with no dynamics, and a test can pause it inside the call, as a lost link does.
// The model serves a constant Jacobian by default, and the FR3 flange kinematics when a test asks for them.

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>

namespace franka {

enum class RealtimeConfig { kEnforce, kIgnore };
enum class Frame { kJoint1, kFlange, kEndEffector, kStiffness };

class Duration {
 public:
  Duration() = default;
  explicit Duration(std::uint64_t milliseconds) : ms_(milliseconds) {}
  double toSec() const { return static_cast<double>(ms_) / 1000.0; }
  Duration& operator+=(const Duration& d) {
    ms_ += d.ms_;
    return *this;
  }

 private:
  std::uint64_t ms_ = 0;
};

class Errors {
 public:
  explicit operator bool() const { return !text_.empty(); }
  explicit operator std::string() const { return text_; }

 private:
  std::string text_;
};

struct RobotState {
  std::array<double, 7> q{0.0, -0.8, 0.0, -2.3, 0.0, 1.6, 0.8};
  std::array<double, 7> dq{};
  std::array<double, 7> q_d{0.0, -0.8, 0.0, -2.3, 0.0, 1.6, 0.8};
  std::array<double, 7> tau_J{};
  std::array<double, 7> tau_J_d{};
  std::array<double, 16> O_T_EE{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, 16> F_T_EE{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::array<double, 6> K_F_ext_hat_K{};
  Duration time;
  Errors current_errors;
};

struct Finishable {
  bool motion_finished = false;
};

struct JointPositions : Finishable {
  JointPositions(const std::array<double, 7>& joint_positions) : q(joint_positions) {}
  std::array<double, 7> q{};
};

struct Torques : Finishable {
  Torques(const std::array<double, 7>& torques) : tau_J(torques) {}
  std::array<double, 7> tau_J{};
};

namespace fake {

enum class Kinematics {
  // pose is the state's O_T_EE, and joint 1 moves every frame 0.5 m/rad along x.
  kConstant,
  // pose and zeroJacobian are those of the FR3 flange at the state's q, for every frame.
  kFr3,
};

inline std::atomic<Kinematics>& kinematics_setting() {
  static std::atomic<Kinematics> setting{Kinematics::kConstant};
  return setting;
}

inline Kinematics kinematics() { return kinematics_setting().load(); }

// The FR3 flange pose and its zero Jacobian (linear rows first), column-major as libfranka's, from Franka's
// modified DH parameters.
struct Fr3Flange {
  std::array<double, 16> pose;
  std::array<double, 42> zero_jacobian;
};

inline Fr3Flange fr3_flange(const std::array<double, 7>& q) {
  constexpr double kHalfPi = 1.57079632679489661923;
  constexpr std::array<double, 8> a = {0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0};
  constexpr std::array<double, 8> d = {0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107};
  constexpr std::array<double, 8> alpha = {0, -kHalfPi, kHalfPi, kHalfPi, -kHalfPi, kHalfPi, kHalfPi, 0};
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  std::array<Eigen::Vector3d, 7> axis, origin;
  for (int i = 0; i < 8; ++i) {
    T = T * Eigen::AngleAxisd(alpha[i], Eigen::Vector3d::UnitX()) * Eigen::Translation3d(a[i], 0, 0) *
        Eigen::AngleAxisd(i < 7 ? q[i] : 0.0, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(0, 0, d[i]);
    if (i < 7) {
      axis[i] = T.linear().col(2);
      origin[i] = T.translation();
    }
  }
  Fr3Flange out{};
  Eigen::Map<Eigen::Matrix4d>(out.pose.data()) = T.matrix();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> J(out.zero_jacobian.data());
  for (int i = 0; i < 7; ++i) {
    J.block<3, 1>(0, i) = axis[i].cross(T.translation() - origin[i]);
    J.block<3, 1>(3, i) = axis[i];
  }
  return out;
}

}  // namespace fake

class Model {
 public:
  std::array<double, 16> pose(Frame, const RobotState& s) const {
    return fake::kinematics() == fake::Kinematics::kFr3 ? fake::fr3_flange(s.q).pose : s.O_T_EE;
  }
  // Column-major 6x7, as libfranka's. kConstant: joint 1 moves every frame 0.5 m/rad along x, and no joint turns it.
  std::array<double, 42> zeroJacobian(Frame, const RobotState& s) const {
    if (fake::kinematics() == fake::Kinematics::kFr3) return fake::fr3_flange(s.q).zero_jacobian;
    std::array<double, 42> J{};
    J[0] = JOINT_1_LEVER_ARM_M;
    return J;
  }
  static constexpr double JOINT_1_LEVER_ARM_M = 0.5;
  std::array<double, 7> coriolis(const RobotState&) const { return {}; }
};

namespace fake {

// The one link every fake Robot uses. A test pauses it to stand for a control call stuck in libfranka.
class Link {
 public:
  void reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    kinematics_setting().store(Kinematics::kConstant);
    state_ = RobotState{};
    paused_ = false;
    control_returns_ = 0;
  }
  // The kinematics of the model, until the next reset. With kFr3, O_T_EE follows q.
  void use_kinematics(Kinematics kinematics) {
    std::lock_guard<std::mutex> lk(mutex_);
    kinematics_setting().store(kinematics);
    follow_kinematics_();
  }
  void pause() {
    std::lock_guard<std::mutex> lk(mutex_);
    paused_ = true;
  }
  void resume() {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      paused_ = false;
    }
    cv_.notify_all();
  }
  RobotState state() {
    std::lock_guard<std::mutex> lk(mutex_);
    return state_;
  }
  int control_returns() {
    std::lock_guard<std::mutex> lk(mutex_);
    return control_returns_;
  }

  template <typename Command>
  void run(const std::function<Command(const RobotState&, Duration)>& callback) {
    Duration period(0);
    while (true) {
      RobotState tick;
      {
        std::unique_lock<std::mutex> lk(mutex_);
        cv_.wait(lk, [this] { return !paused_; });
        tick = state_;
      }
      const Command cmd = callback(tick, period);
      {
        std::lock_guard<std::mutex> lk(mutex_);
        apply_(cmd);
        state_.time += Duration(1);
      }
      if (cmd.motion_finished) break;
      period = Duration(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::lock_guard<std::mutex> lk(mutex_);
    ++control_returns_;
  }

 private:
  void apply_(const JointPositions& cmd) {
    for (size_t i = 0; i < 7; ++i) {
      state_.dq[i] = (cmd.q[i] - state_.q[i]) / 1e-3;
      state_.q[i] = state_.q_d[i] = cmd.q[i];
    }
    follow_kinematics_();
  }
  void follow_kinematics_() {
    if (kinematics() == Kinematics::kFr3) state_.O_T_EE = fr3_flange(state_.q).pose;
  }
  void apply_(const Torques&) {}

  std::mutex mutex_;
  std::condition_variable cv_;
  RobotState state_;
  bool paused_ = false;
  int control_returns_ = 0;
};

inline Link& link() {
  static Link l;
  return l;
}

}  // namespace fake

class Robot {
 public:
  Robot(const std::string&, RealtimeConfig) {}
  Model loadModel() { return Model{}; }
  RobotState readOnce() { return fake::link().state(); }
  void control(std::function<JointPositions(const RobotState&, Duration)> callback) {
    fake::link().run<JointPositions>(callback);
  }
  void control(std::function<Torques(const RobotState&, Duration)> callback, bool, double) {
    fake::link().run<Torques>(callback);
  }
  void setJointImpedance(const std::array<double, 7>&) {}
  void automaticErrorRecovery() {}
  void setCollisionBehavior(const std::array<double, 7>&, const std::array<double, 7>&, const std::array<double, 7>&,
                            const std::array<double, 7>&, const std::array<double, 6>&, const std::array<double, 6>&,
                            const std::array<double, 6>&, const std::array<double, 6>&) {}
  void setLoad(double, const std::array<double, 3>&, const std::array<double, 9>&) {}
  std::string getRobotModel() { return "<robot></robot>"; }
};

}  // namespace franka
