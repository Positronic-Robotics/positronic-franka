#pragma once

// A stand-in for the part of libfranka that src/robot.hpp uses. The control call ticks at 1 ms on a
// joint model with no dynamics, and a test can pause it inside the call, as a lost link does.

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

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

class Model {
 public:
  std::array<double, 16> pose(Frame, const RobotState& s) const { return s.O_T_EE; }
  std::array<double, 42> zeroJacobian(Frame, const RobotState&) const { return {}; }
  std::array<double, 7> coriolis(const RobotState&) const { return {}; }
};

// No speed limit at any position: these tests do not move near a joint limit. test_impedance_law checks the
// reference against the real libfranka limit.
inline std::array<double, 7> computeUpperLimitsJointVelocity(const std::array<double, 7>&) {
  std::array<double, 7> limit;
  limit.fill(std::numeric_limits<double>::infinity());
  return limit;
}

inline std::array<double, 7> computeLowerLimitsJointVelocity(const std::array<double, 7>&) {
  std::array<double, 7> limit;
  limit.fill(-std::numeric_limits<double>::infinity());
  return limit;
}

namespace fake {

// The one link every fake Robot uses. A test pauses it to stand for a control call stuck in libfranka.
class Link {
 public:
  void reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    state_ = RobotState{};
    paused_ = false;
    control_returns_ = 0;
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
