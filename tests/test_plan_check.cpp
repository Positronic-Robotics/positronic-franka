// Checks the position-loop plan check in plan_check.hpp. Needs Eigen, the vendored Ruckig and the shared kinematics
// core, not libfranka. The plan-check job of .github/workflows/tests.yaml builds it.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fr3_with_gripper.hpp"
#include "plan_check.hpp"
#include "trajectory_generator.hpp"
#include "workspace_bounds.hpp"

namespace {

using positronic_franka::SideKind;
using positronic_franka::TrajectoryGenerator;
using positronic_franka::Vector7d;
using positronic_franka::WorkspaceBounds;
namespace sk = shared_kinematics;

struct Failure {
  std::string what;
};

void check(bool ok, const std::string& what) {
  if (!ok) throw Failure{what};
}

constexpr double DYNAMICS_FACTOR = 0.5;
constexpr double DT = TrajectoryGenerator::NOMINAL_DT;
// The start of the fake libfranka. The pad point is 0.32 m ahead of the base.
constexpr std::array<double, 7> START = {0.0, -0.8, 0.0, -2.3, 0.0, 1.6, 0.8};

Vector7d start_q() { return Eigen::Map<const Vector7d>(START.data()); }

Vector7d leaning_forward() {
  Vector7d q = start_q();
  q(1) += 0.8;
  q(3) += 0.4;
  return q;
}

// A front wall `x` ahead of the base.
WorkspaceBounds a_front_wall(double x) {
  return WorkspaceBounds({{"front", SideKind::kSurface, -Eigen::Vector3d::UnitX(), x, 0.0, -0.005}});
}

sk::Guard box_guard(const WorkspaceBounds& bounds) {
  sk::ArmModel model = fr3_with_gripper();
  model.self.pairs.clear();
  return sk::Guard(model, bounds.sides(), positronic_franka::fr3_guard_params());
}

double pad_x(const Vector7d& q) {
  const sk::ArmModel model = fr3_with_gripper();
  const sk::Body& pads = model.bodies.back();
  return (model.chain.frames(q)[pads.capsule.link] * pads.capsule.a).x();
}

Vector7d as_vector(const std::array<double, 7>& a) { return Eigen::Map<const Vector7d>(a.data()); }

TrajectoryGenerator from_rest(bool phase) {
  TrajectoryGenerator traj(DYNAMICS_FACTOR);
  traj.set_phase_synchronization(phase);
  traj.start_at_rest(START);
  traj.step(0.0);
  return traj;
}

// The largest distance of a plan point from the straight joint line of the plan.
double off_the_line(const TrajectoryGenerator& traj, const Vector7d& from, const Vector7d& to) {
  const Vector7d u = (to - from).normalized();
  double worst = 0.0;
  for (double t = 0.0; t <= traj.duration(); t += DT) {
    const Vector7d d = as_vector(traj.position_at(t)) - from;
    worst = std::max(worst, (d - d.dot(u) * u).norm());
  }
  return worst;
}

// The first side that the whole rest of the plan passes, as the position loop checks it.
std::optional<std::string> plan_side(const sk::Guard& box, const TrajectoryGenerator& traj) {
  positronic_franka::PlanCheck plan_check;
  std::optional<std::string> side = plan_check.start(box, traj);
  while (!side && plan_check.pending(traj)) side = plan_check.step(box, traj);
  return side;
}

// The least, over the plan points at `times`, of a part's distance past its floor.
double least_past_floor(const sk::Guard& box, const TrajectoryGenerator& traj, const std::vector<double>& times) {
  double least = std::numeric_limits<double>::infinity();
  for (double t : times)
    for (const auto& row : box.distances(as_vector(traj.position_at(t))).planes)
      least = std::min(least, row.distance - row.floor);
  return least;
}

void a_plan_that_stays_inside_the_box_passes() {
  TrajectoryGenerator traj = from_rest(true);
  check(traj.set_target(leaning_forward()), "the plan is refused");
  check(!plan_side(box_guard(a_front_wall(1.0)), traj), "a plan far from the wall is reported");
}

void a_plan_past_a_side_is_found() {
  TrajectoryGenerator traj = from_rest(true);
  check(traj.set_target(leaning_forward()), "the plan is refused");
  check(pad_x(leaning_forward()) > 0.45, "the target does not take the pad point past 0.45 m");
  const auto side = plan_side(box_guard(a_front_wall(0.42)), traj);
  check(side && *side == "front", "the plan past the front wall is not found");
}

// The floor of a part that the plan starts past is its distance at the start, so a plan away from the side passes.
void a_plan_that_starts_past_a_floor_and_moves_away_passes() {
  TrajectoryGenerator traj(DYNAMICS_FACTOR);
  traj.start_at_rest({START[0], START[1] + 0.8, START[2], START[3] + 0.4, START[4], START[5], START[6]});
  traj.step(0.0);
  check(traj.set_target(start_q()), "the plan is refused");
  check(!plan_side(box_guard(a_front_wall(0.42)), traj), "a plan away from the wall is reported");
}

// The check reads the plan from where it plays: a plan that already passed the side and comes back passes.
void the_check_reads_the_rest_of_the_plan() {
  TrajectoryGenerator traj = from_rest(true);
  check(traj.set_target(leaning_forward()), "the plan is refused");
  const sk::Guard box = box_guard(a_front_wall(0.42));
  while (traj.active()) traj.step(DT);
  check(traj.elapsed() >= traj.duration(), "the plan does not end");
  check(!plan_side(box, traj), "the played part of the plan is checked");
}

void with_phase_synchronization_a_plan_from_rest_follows_the_joint_line() {
  TrajectoryGenerator traj = from_rest(true);
  Vector7d target = leaning_forward();
  target(6) -= 0.3;
  check(traj.set_target(target), "the plan is refused");
  const double off = off_the_line(traj, start_q(), target);
  check(off < 1e-6, "the plan leaves the joint line by " + std::to_string(off) + " rad");
}

// The default, time synchronization, can leave the joint line: the reason the loop checks the plan, not the line.
void with_time_synchronization_a_plan_from_rest_can_leave_the_joint_line() {
  TrajectoryGenerator traj = from_rest(false);
  Vector7d target = leaning_forward();
  target(6) -= 0.3;
  check(traj.set_target(target), "the plan is refused");
  const double off = off_the_line(traj, start_q(), target);
  check(off > 1e-3, "the plan stays on the joint line, off by " + std::to_string(off) + " rad");
}

// A fast move toward the wall that gets a new target where it is now goes past that target before it stops. With the
// wall just past the new target, the check finds the overshoot.
void a_retarget_during_a_fast_move_overshoots_and_the_check_finds_it() {
  TrajectoryGenerator traj = from_rest(false);
  check(traj.set_target(leaning_forward()), "the first plan is refused");
  for (double t = 0.0; t < 0.4 * traj.duration(); t += DT) traj.step(DT);
  const Vector7d now = as_vector(traj.step(DT));
  check(traj.set_target(now), "the retarget is refused");
  const sk::Guard box = box_guard(a_front_wall(pad_x(now) + 0.005));
  double furthest = -std::numeric_limits<double>::infinity();
  for (double t = 0.0; t <= traj.duration(); t += DT)
    furthest = std::max(furthest, pad_x(as_vector(traj.position_at(t))));
  check(furthest > pad_x(now) + 0.01, "the retarget does not overshoot");
  const auto side = plan_side(box, traj);
  check(side && *side == "front", "the overshoot past the wall is not found");
}

// A long plan from rest that a side cuts between 16 evenly spaced points, but not at 10 ms spacing, is found.
void a_pass_between_the_points_of_a_long_plan_is_found() {
  const Eigen::Vector3d n =
      Eigen::Vector3d(-0.75136641891253586, -0.62447579934362674, -0.21325684177697679).normalized();
  Vector7d move;
  move << 0.65984006027525333, 1.0720002528910009, 0.7087466278476825, -1.0388597792839911, -0.24866080884892469,
      -0.24865692807376893, -0.42492166889711741;
  TrajectoryGenerator traj = from_rest(true);
  check(traj.set_target(start_q() + move), "the plan is refused");
  const double duration = traj.duration();
  std::vector<double> every_ms, sixteen;
  for (double t = 0.0; t <= duration; t += DT) every_ms.push_back(t);
  for (int i = 0; i <= 16; ++i) sixteen.push_back(duration * i / 16);
  const auto wall_at = [&](double d) { return WorkspaceBounds({{"cut", SideKind::kSurface, n, d, 0.0, -0.005}}); };
  const double tolerance = positronic_franka::fr3_guard_params().path_tolerance;
  // Move the wall so that the plan passes it by 0.7 mm more than the path tolerance.
  const double d = 2.0 - least_past_floor(box_guard(wall_at(2.0)), traj, every_ms) - tolerance - 0.0007;
  const sk::Guard box = box_guard(wall_at(d));
  check(least_past_floor(box, traj, {0.0}) > 0.0, "the plan does not start inside the wall");
  check(least_past_floor(box, traj, sixteen) >= -tolerance, "16 points of the plan find the pass");
  const auto side = plan_side(box, traj);
  check(side && *side == "cut", "the pass between the points of a long plan is not found");
}

// A retarget during a fast move gets a plan that leaves the joint line that the clip checked. With the self pairs in
// the guard, the check finds the plan where it brings two links closer than the clearance.
void a_retarget_plan_that_brings_two_links_too_close_is_found() {
  Vector7d first, second;
  first << 0.085685566353384424, -0.38185236011868895, -0.56080959107845052, -3.0, -0.014548021326547866,
      1.4355515400008243, 0.86063895399797863;
  second << 0.82167226687014905, -0.62046024570859526, -0.46798736401990615, -1.9897270822297473, 0.45099503027404242,
      0.63539928011235092, 1.334850696122557;
  const sk::Guard& self = positronic_franka::fr3_self_guard();
  TrajectoryGenerator traj(1.0);
  traj.set_phase_synchronization(true);
  traj.start_at_rest(START);
  traj.step(0.0);
  check(traj.set_target(self.clip(start_q(), first).q), "the first plan is refused");
  std::array<double, 7> now{};
  for (int i = 0; i < 258; ++i) now = traj.step(DT);
  check(traj.set_target(self.clip(as_vector(now), second).q), "the retarget is refused");
  const auto side = plan_side(self, traj);
  check(side && *side == positronic_franka::SELF_COLLISION_KEY,
        "the plan that brings two links too close is not found");
}

void a_brake_reports_that_it_is_planned() {
  TrajectoryGenerator traj = from_rest(false);
  check(traj.set_target(leaning_forward()), "the plan is refused");
  for (int i = 0; i < 200; ++i) traj.step(DT);
  check(traj.brake_to_rest(), "the brake is refused");
  while (traj.active()) traj.step(DT);
  check(traj.elapsed() > 0.0, "the brake does not play");
}

// The check runs in the 1 kHz loop: PLAN_CHECK_SAMPLES points when a plan starts, then one point at each tick, ahead of
// the arm. A new plan ends the check of the old one.
void the_check_of_a_long_plan_reads_its_first_points_then_one_point_a_tick() {
  TrajectoryGenerator traj = from_rest(true);
  check(traj.set_target(leaning_forward()), "the plan is refused");
  const int points = static_cast<int>(std::ceil(traj.duration() / positronic_franka::PLAN_CHECK_SPACING_S));
  check(points > 2 * positronic_franka::PLAN_CHECK_SAMPLES, "the plan is too short to test the first points");
  const sk::Guard box = box_guard(a_front_wall(1.0));
  constexpr int kRuns = 100;
  positronic_franka::PlanCheck plan_check;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < kRuns; ++i) check(!plan_check.start(box, traj), "a plan far from the wall is reported");
  const auto t1 = std::chrono::steady_clock::now();
  int ticks = 0;
  while (plan_check.pending(traj)) {
    check(!plan_check.step(box, traj), "a plan far from the wall is reported");
    traj.step(DT);
    ++ticks;
  }
  const auto t2 = std::chrono::steady_clock::now();
  check(ticks == points - positronic_franka::PLAN_CHECK_SAMPLES,
        "the check reads " + std::to_string(ticks) + " points after the start, not " +
            std::to_string(points - positronic_franka::PLAN_CHECK_SAMPLES));
  std::cout << "  plan check: start " << std::chrono::duration<double, std::micro>(t1 - t0).count() / kRuns
            << " us, tick " << std::chrono::duration<double, std::micro>(t2 - t1).count() / ticks << " us\n";
  TrajectoryGenerator braked = from_rest(true);
  check(braked.set_target(leaning_forward()), "the plan is refused");
  check(!plan_check.start(box, braked) && plan_check.pending(braked), "the check reads the whole plan at its start");
  for (int i = 0; i < 100; ++i) braked.step(DT);
  check(braked.brake_to_rest(), "the brake is refused");
  check(!plan_check.pending(braked), "the check of the old plan goes on after the brake");
}

}  // namespace

int main() {
  const std::vector<std::pair<std::string, std::function<void()>>> tests = {
      {"a_plan_that_stays_inside_the_box_passes", a_plan_that_stays_inside_the_box_passes},
      {"a_plan_past_a_side_is_found", a_plan_past_a_side_is_found},
      {"a_plan_that_starts_past_a_floor_and_moves_away_passes", a_plan_that_starts_past_a_floor_and_moves_away_passes},
      {"the_check_reads_the_rest_of_the_plan", the_check_reads_the_rest_of_the_plan},
      {"with_phase_synchronization_a_plan_from_rest_follows_the_joint_line",
       with_phase_synchronization_a_plan_from_rest_follows_the_joint_line},
      {"with_time_synchronization_a_plan_from_rest_can_leave_the_joint_line",
       with_time_synchronization_a_plan_from_rest_can_leave_the_joint_line},
      {"a_retarget_during_a_fast_move_overshoots_and_the_check_finds_it",
       a_retarget_during_a_fast_move_overshoots_and_the_check_finds_it},
      {"a_pass_between_the_points_of_a_long_plan_is_found", a_pass_between_the_points_of_a_long_plan_is_found},
      {"a_retarget_plan_that_brings_two_links_too_close_is_found",
       a_retarget_plan_that_brings_two_links_too_close_is_found},
      {"a_brake_reports_that_it_is_planned", a_brake_reports_that_it_is_planned},
      {"the_check_of_a_long_plan_reads_its_first_points_then_one_point_a_tick",
       the_check_of_a_long_plan_reads_its_first_points_then_one_point_a_tick},
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
  std::cout << (tests.size() - failed) << "/" << tests.size() << " passed\n";
  return failed == 0 ? 0 : 1;
}
