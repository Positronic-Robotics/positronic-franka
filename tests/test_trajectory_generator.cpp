// Checks the joint trajectory generator in trajectory_generator.hpp. Needs Eigen and the vendored Ruckig, not libfranka.
// Ruckig needs C++20. Build it once, then this test:
//   for f in $(ls external/ruckig/src/ruckig/*.cpp | grep -v -e cloud_client -e python); do
//     g++ -std=c++20 -O2 -I external/ruckig/include -c $f -o $(basename $f .cpp).o; done
//   g++ -std=c++20 -I src -I external/ruckig/include $(pkg-config --cflags eigen3) tests/test_trajectory_generator.cpp *.o
//   ./a.out

#include <array>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <random>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "trajectory_generator.hpp"

namespace {

using positronic_franka::PANDA_BASE_ACCELERATION_LIMITS;
using positronic_franka::PANDA_BASE_JERK_LIMITS;
using positronic_franka::PANDA_BASE_VELOCITY_LIMITS;
using positronic_franka::TrajectoryGenerator;
using positronic_franka::Vector7d;

struct Failure {
  std::string what;
};

void check(bool ok, const std::string& what) {
  if (!ok) throw Failure{what};
}

constexpr double DYNAMICS_FACTOR = 0.2;
constexpr std::array<double, 7> PARK = {0.0, -0.31, 0.0, -1.65, 0.0, 1.522, 0.0};

Vector7d park_plus(double d) {
  Vector7d target = Eigen::Map<const Vector7d>(PARK.data());
  target.array() += d;
  return target;
}

double max_offset(const std::array<double, 7>& a, const std::array<double, 7>& b) {
  double worst = 0.0;
  for (size_t i = 0; i < 7; ++i) worst = std::max(worst, std::abs(a[i] - b[i]));
  return worst;
}

// libfranka passes a zero period on the first tick; that command must equal the start position.
void the_first_tick_of_a_move_commands_the_start_position() {
  TrajectoryGenerator traj(DYNAMICS_FACTOR);
  traj.start_at_rest(PARK);
  check(traj.set_target(park_plus(0.4)), "plan refused");
  check(max_offset(traj.step(0.0), PARK) == 0.0, "the zero-period tick moved off the start position");
}

// A move starts at rest, so the first 1 ms covers at most jerk * t^3 / 6 on each joint.
void the_first_millisecond_of_a_move_is_jerk_limited_from_rest() {
  TrajectoryGenerator traj(DYNAMICS_FACTOR);
  traj.start_at_rest(PARK);
  check(traj.set_target(park_plus(0.4)), "plan refused");
  traj.step(0.0);
  const double dt = 0.001;
  const auto pos = traj.step(dt);
  for (size_t i = 0; i < 7; ++i) {
    const double bound = PANDA_BASE_JERK_LIMITS[i] * DYNAMICS_FACTOR * dt * dt * dt / 6.0 + 1e-12;
    check(std::abs(pos[i] - PARK[i]) <= bound, "joint " + std::to_string(i + 1) + " moved faster than from rest");
  }
}

void a_move_from_rest_reaches_its_target_and_ends() {
  TrajectoryGenerator traj(DYNAMICS_FACTOR);
  traj.start_at_rest(PARK);
  const Vector7d target = park_plus(0.4);
  check(traj.set_target(target), "plan refused");
  std::array<double, 7> pos = traj.step(0.0);
  for (int tick = 0; tick < 100000 && traj.active(); ++tick) pos = traj.step(0.001);
  check(!traj.active(), "the move did not end");
  for (size_t i = 0; i < 7; ++i) check(std::abs(pos[i] - target[i]) < 1e-9, "the move ended off its target");
}

void a_target_at_the_start_position_plans_and_holds() {
  TrajectoryGenerator traj(DYNAMICS_FACTOR);
  traj.start_at_rest(PARK);
  check(traj.set_target(park_plus(0.0)), "plan refused");
  check(max_offset(traj.step(0.001), PARK) == 0.0, "the arm moved on a target at the start position");
}

Vector7d random_target(std::mt19937& rng) {
  std::uniform_real_distribution<> offset(-0.8, 0.8);
  Vector7d target = Eigen::Map<const Vector7d>(PARK.data());
  for (int i = 0; i < 7; ++i) target[i] += offset(rng);
  return target;
}

// Plays the current plan to its end and returns where it stops.
std::array<double, 7> play_out(TrajectoryGenerator& traj) {
  std::array<double, 7> pos = traj.step(0.0);
  for (int tick = 0; tick < 100000 && traj.active(); ++tick) pos = traj.step(0.001);
  check(!traj.active(), "the plan did not end");
  return pos;
}

// The time a joint needs to stop from top speed: down the acceleration at the acceleration limit, plus one jerk ramp.
double longest_brake_s() {
  double worst = 0.0;
  for (size_t i = 0; i < 7; ++i) {
    worst = std::max(worst, PANDA_BASE_VELOCITY_LIMITS[i] / PANDA_BASE_ACCELERATION_LIMITS[i] +
                                PANDA_BASE_ACCELERATION_LIMITS[i] / PANDA_BASE_JERK_LIMITS[i]);
  }
  return worst;
}

// A refused brake leaves the old plan playing, so the arm goes on to the old target for seconds.
void a_brake_mid_move_brings_the_arm_to_rest_in_a_brake_time() {
  std::mt19937 rng(7);
  std::uniform_int_distribution<> brake_tick(1, 1500);
  for (int k = 0; k < 200; ++k) {
    TrajectoryGenerator traj(DYNAMICS_FACTOR);
    traj.start_at_rest(PARK);
    check(traj.set_target(random_target(rng)), "plan refused");
    traj.step(0.0);
    for (int i = brake_tick(rng); i > 0 && traj.active(); --i) traj.step(0.001);
    traj.brake_to_rest();
    check(traj.duration() <= longest_brake_s(), "the brake runs longer than a stop from top speed");
    const auto rest = play_out(traj);
    check(max_offset(traj.step(0.001), rest) == 0.0, "the arm moved after the brake ended");
  }
}

void a_brake_after_a_move_ends_holds_the_arm_where_it_is() {
  std::mt19937 rng(11);
  for (int k = 0; k < 200; ++k) {
    TrajectoryGenerator traj(DYNAMICS_FACTOR);
    traj.start_at_rest(PARK);
    check(traj.set_target(random_target(rng)), "plan refused");
    const auto end = play_out(traj);
    traj.brake_to_rest();
    check(max_offset(play_out(traj), end) < 1e-9, "the brake moved a resting arm");
  }
}

}  // namespace

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
      {"the_first_tick_of_a_move_commands_the_start_position", the_first_tick_of_a_move_commands_the_start_position},
      {"the_first_millisecond_of_a_move_is_jerk_limited_from_rest",
       the_first_millisecond_of_a_move_is_jerk_limited_from_rest},
      {"a_move_from_rest_reaches_its_target_and_ends", a_move_from_rest_reaches_its_target_and_ends},
      {"a_target_at_the_start_position_plans_and_holds", a_target_at_the_start_position_plans_and_holds},
      {"a_brake_mid_move_brings_the_arm_to_rest_in_a_brake_time",
       a_brake_mid_move_brings_the_arm_to_rest_in_a_brake_time},
      {"a_brake_after_a_move_ends_holds_the_arm_where_it_is", a_brake_after_a_move_ends_holds_the_arm_where_it_is},
  };
  int failed = 0;
  for (const auto& [name, test] : tests) {
    try {
      test();
      std::cout << "PASS " << name << "\n";
    } catch (const Failure& f) {
      ++failed;
      std::cout << "FAIL " << name << ": " << f.what << "\n";
    }
  }
  return failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
