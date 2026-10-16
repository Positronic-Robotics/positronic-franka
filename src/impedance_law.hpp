#pragma once

#include <algorithm>
#include <array>

#include <Eigen/Core>

namespace positronic_franka {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

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

// The software impedance law: tau = Kp (q_d - q) - Kd dq + coriolis. Only the spring term goes
// through the limiter; damping and Coriolis pass through as computed.
inline Vector7d impedance_torque(const Matrix7d& Kp, const Matrix7d& Kd, const Vector7d& q_d, const Vector7d& q,
                                 const Vector7d& dq, const Vector7d& coriolis, SpringTorqueRateLimiter& spring_limiter,
                                 double dt) {
  return spring_limiter.limit(Kp * (q_d - q), dt) - Kd * dq + coriolis;
}

}  // namespace positronic_franka
