#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>
#include <franka/rate_limiting.h>

namespace positronic_franka {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

// The share of the FR3 position-dependent joint speed limit that the reference q_d may use. The arm lags q_d, and its
// speed can pass the speed of q_d for a moment, so the rest of the limit is room for that lag.
constexpr double REFERENCE_SHARE_OF_FR3_SPEED_LIMIT = 0.8;

// Caps how fast the spring torque Kp (q_d - q) may change, per joint, in Nm/s. It starts from zero
// spring torque, which is the torque at q_d = q, where every control loop starts.
class SpringTorqueRateLimiter {
 public:
  explicit SpringTorqueRateLimiter(const std::array<double, 7>& max_rate) { set_max_rate(max_rate); }

  void set_max_rate(const std::array<double, 7>& max_rate) { max_rate_ = Eigen::Map<const Vector7d>(max_rate.data()); }

  Vector7d limit(const Vector7d& spring, double dt) {
    for (int i = 0; i < 7; ++i) {
      const double step = max_rate_(i) * dt;
      last_(i) = std::clamp(spring(i), last_(i) - step, last_(i) + step);
    }
    ramping_ = last_ != spring;
    return last_;
  }

  // True while the last call returned less than the spring torque it was given.
  bool ramping() const { return ramping_; }

 private:
  Vector7d max_rate_;
  Vector7d last_ = Vector7d::Zero();
  bool ramping_ = false;
};

// A stop is over once the arm is quiet and the capped spring torque has ramped out; a still arm with
// spring torque left in the ramp would drop that torque at once when the loop ends.
inline bool stop_has_settled(const SpringTorqueRateLimiter& spring_limiter, const Vector7d& dq,
                             double velocity_tolerance) {
  return !spring_limiter.ramping() && dq.cwiseAbs().maxCoeff() < velocity_tolerance;
}

// The speed (rad/s, per joint) at which the reference may move from `ref` toward `target`: `max_speed`, and at most
// REFERENCE_SHARE_OF_FR3_SPEED_LIMIT of the FR3 limit in the direction of the target. The FR3 limit falls to zero near
// each joint position limit. libfranka (franka/rate_limiting.h) applies it to its position and velocity generators,
// not to a torque command, so the torque loop applies it here.
inline Vector7d reference_speed_toward(const Vector7d& ref, const Vector7d& target, const Vector7d& max_speed) {
  std::array<double, 7> q;
  Eigen::Map<Vector7d>(q.data()) = ref;
  const std::array<double, 7> upper = franka::computeUpperLimitsJointVelocity(q);
  const std::array<double, 7> lower = franka::computeLowerLimitsJointVelocity(q);
  Vector7d speed;
  for (int i = 0; i < 7; ++i) {
    const double fr3_limit = target(i) >= ref(i) ? upper[i] : -lower[i];
    speed(i) = std::clamp(REFERENCE_SHARE_OF_FR3_SPEED_LIMIT * fr3_limit, 0.0, max_speed(i));
  }
  return speed;
}

// Moves the spring reference q_d toward `target` in a straight line in joint space, as far as
// reference_speed_toward allows in `dt`. All joints arrive together, so a joint near its position limit slows the
// whole line. The result is `target` once it is within one step.
inline Vector7d move_reference_toward(const Vector7d& ref, const Vector7d& target, const Vector7d& max_speed,
                                      double dt) {
  const Vector7d remaining = target - ref;
  const Vector7d speed = reference_speed_toward(ref, target, max_speed);
  double fraction = 1.0;
  for (int i = 0; i < 7; ++i) {
    const double step = speed(i) * dt;
    if (std::abs(remaining(i)) > step) fraction = std::min(fraction, step / std::abs(remaining(i)));
  }
  return fraction == 1.0 ? target : Vector7d(ref + fraction * remaining);
}

// The software impedance law: tau = Kp (q_d - q) - Kd dq + coriolis. Only the spring term goes
// through the limiter; damping and Coriolis pass through as computed.
inline Vector7d impedance_torque(const Matrix7d& Kp, const Matrix7d& Kd, const Vector7d& q_d, const Vector7d& q,
                                 const Vector7d& dq, const Vector7d& coriolis, SpringTorqueRateLimiter& spring_limiter,
                                 double dt) {
  return spring_limiter.limit(Kp * (q_d - q), dt) - Kd * dq + coriolis;
}

}  // namespace positronic_franka
