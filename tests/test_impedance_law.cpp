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
using positronic_franka::move_reference_toward;
using positronic_franka::stop_has_settled;
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
  SpringTorqueRateLimiter limiter(uniform(rate));
  limiter.limit(Kp * (q - q), dt);  // at rest at q

  const Vector7d full = Kp * (target - q);
  check((full.cwiseAbs().array() > 10.0 * rate * dt).all(), "the step is many cap steps away");
  Vector7d previous = Vector7d::Zero();
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

void a_target_on_the_first_tick_ramps_from_zero_spring_torque() {
  const double rate = 100.0;
  const double dt = 1e-3;
  SpringTorqueRateLimiter limiter(uniform(rate));
  const Vector7d full = stiffness() * Vector7d::Constant(0.3);
  const Vector7d first = limiter.limit(full, dt);
  for (int i = 0; i < 7; ++i) {
    check(near(first(i), std::copysign(rate * dt, full(i))),
          "joint " + std::to_string(i) + " moves one step from zero");
  }
}

void damping_and_coriolis_pass_through_uncapped() {
  const Matrix7d Kp = stiffness();
  const Matrix7d Kd = damping();
  const Vector7d q = Vector7d::Constant(-0.4);
  const Vector7d q_d = q;  // no spring torque, so the cap has nothing to hold back
  const Vector7d coriolis = Vector7d::LinSpaced(-2.0, 3.0);
  SpringTorqueRateLimiter limiter(uniform(1.0));
  impedance_torque(Kp, Kd, q_d, q, Vector7d::Zero(), Vector7d::Zero(), limiter, 0.0);

  // A velocity step makes a damping step far past the 1 Nm/s cap.
  const Vector7d dq = Vector7d::Constant(1.5);
  const Vector7d tau = impedance_torque(Kp, Kd, q_d, q, dq, coriolis, limiter, 1e-3);
  check(tau == -(Kd * dq) + coriolis, "damping and coriolis arrive in full on the first tick");
}

void a_cap_above_the_spring_torque_change_leaves_the_law_unchanged() {
  const Matrix7d Kp = stiffness();
  const Matrix7d Kd = damping();
  SpringTorqueRateLimiter limiter(uniform(1000.0));  // 1 Nm per 1 ms tick
  Vector7d q = Vector7d::LinSpaced(-1.1, 1.3);
  Vector7d dq = Vector7d::LinSpaced(0.7, -0.2);
  const Vector7d coriolis = Vector7d::LinSpaced(0.01, 0.09);
  Vector7d previous_spring = Vector7d::Zero();
  for (int tick = 0; tick < 50; ++tick) {
    const Vector7d q_d = q + Vector7d::Constant(0.0001 * (tick % 5));
    const Vector7d spring = Kp * (q_d - q);
    if (tick > 0) check((spring - previous_spring).cwiseAbs().maxCoeff() < 1.0, "the change stays inside the cap");
    previous_spring = spring;
    const Vector7d tau = impedance_torque(Kp, Kd, q_d, q, dq, coriolis, limiter, 1e-3);
    const Vector7d uncapped = Kp * (q_d - q) - Kd * dq + coriolis;
    check(tau == uncapped, "tick " + std::to_string(tick) + " equals the uncapped law exactly");
    q += 1e-3 * dq;
    dq *= 0.97;
  }
}

void a_cap_change_mid_session_ramps_from_the_last_spring_torque() {
  const Matrix7d Kp = stiffness();
  SpringTorqueRateLimiter limiter(uniform(1000.0));
  const Vector7d before = limiter.limit(Kp * Vector7d::Constant(0.1), 1e-3);
  limiter.set_max_rate(uniform(50.0));
  const Vector7d after = limiter.limit(Kp * Vector7d::Constant(0.5), 1e-3);
  check(((after - before).array().abs() - 50.0 * 1e-3).abs().maxCoeff() < 1e-9,
        "the first tick at the new cap moves one new step from the torque the arm already had");
}

void a_stop_with_a_still_arm_settles_only_once_the_spring_torque_ramps_to_zero() {
  const double tolerance = 0.05;  // rad/s
  const Vector7d still = Vector7d::Zero();
  SpringTorqueRateLimiter limiter(uniform(35.0));
  for (int tick = 0; tick < 2000; ++tick) limiter.limit(Vector7d::Constant(10.0), 1e-3);  // held at 10 Nm

  // The stop sets q_d = q, so the spring torque the limiter is given drops to zero.
  int ticks = 0;
  for (; ticks < 2000; ++ticks) {
    const Vector7d spring = limiter.limit(Vector7d::Zero(), 1e-3);
    if (stop_has_settled(limiter, still, tolerance)) {
      check(spring.isZero(), "a settled stop holds no spring torque");
      break;
    }
  }
  check(ticks >= 285, "10 Nm at 35 Nm/s takes about 286 ticks to ramp out, it took " + std::to_string(ticks));
  check(ticks < 2000, "the stop settles once the ramp ends");
  check(!stop_has_settled(limiter, Vector7d::Constant(0.2), tolerance), "a moving arm has not settled");
}

const Vector7d REFERENCE_SPEED = (Vector7d() << 1.5, 1.5, 1.5, 1.5, 1.2, 1.0, 2.0).finished();

// A jump of 2.4 rad on joint 5 against 0.3 rad or less elsewhere: joint 5 at 1.2 rad/s sets the pace.
Vector7d far_target(const Vector7d& start) {
  Vector7d jump;
  jump << 0.3, -0.2, 0.1, 0.0, -2.4, 0.05, 0.25;
  return start + jump;
}

void the_reference_moves_to_a_far_target_at_the_capped_speed_on_a_straight_line() {
  const Vector7d start = Vector7d::LinSpaced(-1.1, 1.3);
  const Vector7d target = far_target(start);
  const double dt = 1e-3;
  Vector7d ref = start;
  for (int tick = 1; tick <= 1000; ++tick) {
    const Vector7d next = move_reference_toward(ref, target, REFERENCE_SPEED, dt);
    check(((next - ref).cwiseAbs().array() <= REFERENCE_SPEED.array() * dt + 1e-12).all(),
          "tick " + std::to_string(tick) + ": a joint moved faster than its cap");
    ref = next;
  }
  // After 1 s joint 5 has moved 1.2 rad, half of its 2.4 rad, and every other joint half of its jump.
  check(((ref - (start + 0.5 * (target - start))).cwiseAbs().array() < 1e-9).all(),
        "after 1 s every joint is halfway along the line");
}

void a_far_target_is_reached_exactly_when_the_slowest_joint_arrives() {
  const Vector7d start = Vector7d::LinSpaced(-1.1, 1.3);
  const Vector7d target = far_target(start);
  const double dt = 1e-3;
  Vector7d ref = start;
  int ticks = 0;
  while (ref != target && ticks < 10000) {
    ref = move_reference_toward(ref, target, REFERENCE_SPEED, dt);
    ++ticks;
  }
  check(ref == target, "the reference reaches the target bit for bit");
  // 2.4 rad at 1.2 rad/s is 2 s, so 2000 ticks; one more at most for rounding.
  check(ticks >= 2000 && ticks <= 2001, "it took " + std::to_string(ticks) + " ticks, not 2000");
}

void a_target_within_one_step_is_reached_on_that_tick() {
  const Vector7d start = Vector7d::LinSpaced(-1.1, 1.3);
  const Vector7d target = start + Vector7d::Constant(0.0009);  // below 1.0 rad/s * 1 ms on every joint
  check(move_reference_toward(start, target, REFERENCE_SPEED, 1e-3) == target, "a small step lands on the target");
}

void a_new_target_mid_move_turns_the_reference_from_where_it_is() {
  const Vector7d start = Vector7d::Zero();
  Vector7d ref = start;
  for (int tick = 0; tick < 100; ++tick) ref = move_reference_toward(ref, far_target(start), REFERENCE_SPEED, 1e-3);
  const Vector7d turned_at = ref;
  const Vector7d next = move_reference_toward(ref, start, REFERENCE_SPEED, 1e-3);
  check(((next - turned_at).cwiseAbs().array() <= REFERENCE_SPEED.array() * 1e-3 + 1e-12).all(),
        "the turn does not jump the reference");
  check((next - start).norm() < (turned_at - start).norm(), "the reference heads back to the new target");
}

void a_zero_period_leaves_the_reference_where_it_is() {
  const Vector7d start = Vector7d::LinSpaced(-1.1, 1.3);
  check(move_reference_toward(start, far_target(start), REFERENCE_SPEED, 0.0) == start, "no time, no move");
}

}  // namespace

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
      {"a_target_step_ramps_the_spring_torque_at_the_cap_rate", a_target_step_ramps_the_spring_torque_at_the_cap_rate},
      {"a_target_on_the_first_tick_ramps_from_zero_spring_torque",
       a_target_on_the_first_tick_ramps_from_zero_spring_torque},
      {"damping_and_coriolis_pass_through_uncapped", damping_and_coriolis_pass_through_uncapped},
      {"a_cap_above_the_spring_torque_change_leaves_the_law_unchanged",
       a_cap_above_the_spring_torque_change_leaves_the_law_unchanged},
      {"a_cap_change_mid_session_ramps_from_the_last_spring_torque",
       a_cap_change_mid_session_ramps_from_the_last_spring_torque},
      {"a_stop_with_a_still_arm_settles_only_once_the_spring_torque_ramps_to_zero",
       a_stop_with_a_still_arm_settles_only_once_the_spring_torque_ramps_to_zero},
      {"the_reference_moves_to_a_far_target_at_the_capped_speed_on_a_straight_line",
       the_reference_moves_to_a_far_target_at_the_capped_speed_on_a_straight_line},
      {"a_far_target_is_reached_exactly_when_the_slowest_joint_arrives",
       a_far_target_is_reached_exactly_when_the_slowest_joint_arrives},
      {"a_target_within_one_step_is_reached_on_that_tick", a_target_within_one_step_is_reached_on_that_tick},
      {"a_new_target_mid_move_turns_the_reference_from_where_it_is",
       a_new_target_mid_move_turns_the_reference_from_where_it_is},
      {"a_zero_period_leaves_the_reference_where_it_is", a_zero_period_leaves_the_reference_where_it_is},
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
