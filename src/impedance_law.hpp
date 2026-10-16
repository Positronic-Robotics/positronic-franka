#pragma once

#include <algorithm>
#include <array>
#include <optional>

#include <Eigen/Core>

namespace positronic_franka {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

// Caps how fast the spring torque Kp (q_d - q) may change, per joint, in Nm/s. With no cap set it
// passes the spring torque through unchanged. It always keeps the last torque it returned, so a cap
// set mid-session ramps from the torque the arm already gets.
class SpringTorqueRateLimiter {
 public:
  void set_max_rate(const std::optional<std::array<double, 7>>& max_rate) {
    max_rate_ = max_rate ? std::optional<Vector7d>(Eigen::Map<const Vector7d>(max_rate->data())) : std::nullopt;
  }

  Vector7d limit(const Vector7d& spring, double dt) {
    Vector7d out = spring;
    if (max_rate_ && last_) {
      for (int i = 0; i < 7; ++i) {
        const double step = (*max_rate_)(i) * dt;
        out(i) = std::clamp(spring(i), (*last_)(i) - step, (*last_)(i) + step);
      }
    }
    last_ = out;
    return out;
  }

 private:
  std::optional<Vector7d> max_rate_;
  std::optional<Vector7d> last_;
};

// The software impedance law: tau = Kp (q_d - q) - Kd dq + coriolis. Only the spring term goes
// through the limiter; damping and Coriolis pass through as computed.
inline Vector7d impedance_torque(const Matrix7d& Kp, const Matrix7d& Kd, const Vector7d& q_d, const Vector7d& q,
                                 const Vector7d& dq, const Vector7d& coriolis, SpringTorqueRateLimiter& spring_limiter,
                                 double dt) {
  return spring_limiter.limit(Kp * (q_d - q), dt) - Kd * dq + coriolis;
}

}  // namespace positronic_franka
