// Checks the software impedance law in impedance_law.hpp. Needs only Eigen, not libfranka:
//   g++ -std=c++17 -I src $(pkg-config --cflags eigen3) tests/test_impedance_law.cpp -o test_impedance_law
//   ./test_impedance_law

#include <array>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "impedance_law.hpp"

namespace {

using positronic_franka::impedance_torque;
using positronic_franka::Matrix7d;
using positronic_franka::SpringTorqueRateLimiter;
using positronic_franka::Vector7d;

struct Failure {
  std::string what;
};

void check(bool ok, const std::string& what) {
  if (!ok) throw Failure{what};
}

bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

// A dense, non-symmetric Kp and Kd, so that each joint sees the coupling a Cartesian half adds.
Matrix7d stiffness() {
  Matrix7d K = Matrix7d::Constant(3.7);
  K.diagonal() << 40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0;
  K(0, 3) = -11.3;
  return K;
}

Matrix7d damping() {
  Matrix7d K = Matrix7d::Constant(0.6);
  K.diagonal() << 4.0, 6.0, 5.0, 5.0, 3.0, 2.0, 1.0;
  K(2, 5) = -1.9;
  return K;
}

std::array<double, 7> uniform(double v) {
  std::array<double, 7> a;
  a.fill(v);
  return a;
}

void a_target_step_ramps_the_spring_torque_at_the_cap_rate() {
  const Matrix7d Kp = stiffness();
  const Vector7d q = Vector7d::Constant(0.2);
  const Vector7d target = q + Vector7d::Constant(0.3);
  const double rate = 100.0;  // Nm/s
  const double dt = 1.3e-3;   // not 1 ms, so a fixed-period step would show
  SpringTorqueRateLimiter limiter;
  limiter.set_max_rate(uniform(rate));

  // At rest at q: the first tick passes the zero spring torque through and becomes the state.
  const Vector7d at_rest = limiter.limit(Kp * (q - q), 0.0);
  check(at_rest.isZero(), "first tick passes the spring torque through");

  const Vector7d full = Kp * (target - q);
  check((full.cwiseAbs().array() > 10.0 * rate * dt).all(), "the step is many cap steps away");
  Vector7d previous = at_rest;
  for (int tick = 1; tick <= 5; ++tick) {
    const Vector7d spring = limiter.limit(full, dt);
    for (int i = 0; i < 7; ++i) {
      check(near(spring(i) - previous(i), std::copysign(rate * dt, full(i))),
            "tick " + std::to_string(tick) + " joint " + std::to_string(i) + " moves by rate * dt");
    }
    previous = spring;
  }

  // Once inside one step of the target torque it lands on it exactly.
  for (int tick = 0; tick < 10000 && previous != full; ++tick) previous = limiter.limit(full, dt);
  check(previous == full, "the ramp ends on the uncapped spring torque");
}

void damping_and_coriolis_pass_through_uncapped() {
  const Matrix7d Kp = stiffness();
  const Matrix7d Kd = damping();
  const Vector7d q = Vector7d::Constant(-0.4);
  const Vector7d q_d = q;  // no spring torque, so the cap has nothing to hold back
  const Vector7d coriolis = Vector7d::LinSpaced(-2.0, 3.0);
  SpringTorqueRateLimiter limiter;
  limiter.set_max_rate(uniform(1.0));
  impedance_torque(Kp, Kd, q_d, q, Vector7d::Zero(), Vector7d::Zero(), limiter, 0.0);

  // A velocity step makes a damping step far past the 1 Nm/s cap.
  const Vector7d dq = Vector7d::Constant(1.5);
  const Vector7d tau = impedance_torque(Kp, Kd, q_d, q, dq, coriolis, limiter, 1e-3);
  check(tau == -(Kd * dq) + coriolis, "damping and coriolis arrive in full on the first tick");
}

void no_cap_matches_the_uncapped_law_bit_for_bit() {
  const Matrix7d Kp = stiffness();
  const Matrix7d Kd = damping();
  SpringTorqueRateLimiter limiter;  // no cap set
  Vector7d q = Vector7d::LinSpaced(-1.1, 1.3);
  Vector7d dq = Vector7d::LinSpaced(0.7, -0.2);
  const Vector7d coriolis = Vector7d::LinSpaced(0.01, 0.09);
  for (int tick = 0; tick < 50; ++tick) {
    // Targets jump every few ticks, as streamed targets do.
    const Vector7d q_d = q + Vector7d::Constant(tick % 7 == 0 ? 0.9 : 0.013 * tick);
    const Vector7d tau = impedance_torque(Kp, Kd, q_d, q, dq, coriolis, limiter, 1e-3);
    const Vector7d uncapped = Kp * (q_d - q) - Kd * dq + coriolis;
    check(tau == uncapped, "tick " + std::to_string(tick) + " equals the uncapped law exactly");
    q += 1e-3 * dq;
    dq *= 0.97;
  }
}

void a_cap_set_mid_session_ramps_from_the_last_spring_torque() {
  const Matrix7d Kp = stiffness();
  const Vector7d q = Vector7d::Zero();
  SpringTorqueRateLimiter limiter;
  const Vector7d before = limiter.limit(Kp * Vector7d::Constant(0.1), 1e-3);  // no cap yet
  limiter.set_max_rate(uniform(50.0));
  const Vector7d after = limiter.limit(Kp * Vector7d::Constant(0.5), 1e-3);
  check(((after - before).array().abs() - 50.0 * 1e-3).abs().maxCoeff() < 1e-9,
        "the first capped tick moves one step from the torque the arm already had");
  limiter.set_max_rate(std::nullopt);
  const Vector7d released = limiter.limit(Kp * (Vector7d::Constant(0.5) - q), 1e-3);
  check(released == Kp * (Vector7d::Constant(0.5) - q), "clearing the cap passes the spring torque through");
}

}  // namespace

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
      {"a_target_step_ramps_the_spring_torque_at_the_cap_rate", a_target_step_ramps_the_spring_torque_at_the_cap_rate},
      {"damping_and_coriolis_pass_through_uncapped", damping_and_coriolis_pass_through_uncapped},
      {"no_cap_matches_the_uncapped_law_bit_for_bit", no_cap_matches_the_uncapped_law_bit_for_bit},
      {"a_cap_set_mid_session_ramps_from_the_last_spring_torque",
       a_cap_set_mid_session_ramps_from_the_last_spring_torque},
  };
  int failed = 0;
  for (const auto& [name, test] : tests) {
    try {
      test();
      std::cout << "PASS " << name << "\n";
    } catch (const Failure& f) {
      std::cout << "FAIL " << name << ": " << f.what << "\n";
      ++failed;
    }
  }
  return failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
