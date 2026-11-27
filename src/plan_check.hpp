#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <shared_kinematics/shared_kinematics.hpp>

#include "impedance_law.hpp"
#include "trajectory_generator.hpp"
#include "workspace_bounds.hpp"

namespace positronic_franka {

// The position loop checks a plan at points this far apart in time, and the path of a brake from each of them:
// PLAN_CHECK_SAMPLES points and PLAN_CHECK_BRAKES brakes when the plan starts, then one more of each at each tick, so
// the check stays ahead of the arm.
constexpr double PLAN_CHECK_SPACING_S = 0.01;
constexpr int PLAN_CHECK_SAMPLES = 16;
constexpr int PLAN_CHECK_BRAKES = 4;

// Where the plan check finds a bounded part past its floor at a side, or two links closer than the clearance.
struct PlanPass {
  // The side, or SELF_COLLISION_KEY.
  std::string side;
  // Whether it is on a brake from a point of the plan, not at a point of the plan.
  bool on_a_brake = false;
};

// The check of the playing plan against the sides and the self pairs of a guard: its points, and the path of a brake
// from each of them. The floor of each check is its floor, or its value where the check started when that is
// less, less the path tolerance.
class PlanCheck {
 public:
  // Starts the check of the plan that `traj` plays now, from where it is, and checks its first points and brakes.
  std::optional<PlanPass> start(const shared_kinematics::Guard& guard, const TrajectoryGenerator& traj) {
    plan_ = traj.plan_number();
    next_point_ = traj.elapsed();
    next_brake_ = traj.elapsed();
    start_ = distances_at_(guard, traj.position_at(next_point_));
    std::optional<PlanPass> pass;
    for (int i = 0; i < PLAN_CHECK_SAMPLES && !pass && next_point_ < traj.duration(); ++i)
      pass = check_next_point_(guard, traj);
    for (int i = 0; i < PLAN_CHECK_BRAKES && !pass && next_brake_ < traj.duration(); ++i)
      pass = check_next_brake_(guard, traj);
    return pass;
  }

  // Whether points of the plan that the check started on, or brakes from them, are left to check.
  bool pending(const TrajectoryGenerator& traj) const {
    return plan_ != 0 && traj.plan_number() == plan_ &&
           (next_point_ < traj.duration() || next_brake_ < traj.duration());
  }

  // Checks the next point of the plan, and the path of a brake from the next point whose brake is not checked. A brake
  // that the planner refuses passes the nearest floor.
  std::optional<PlanPass> step(const shared_kinematics::Guard& guard, const TrajectoryGenerator& traj) {
    if (next_point_ < traj.duration())
      if (std::optional<PlanPass> pass = check_next_point_(guard, traj)) return pass;
    if (next_brake_ >= traj.duration()) return std::nullopt;
    return check_next_brake_(guard, traj);
  }

  // Whether a brake from where the plan is now stays above each floor.
  bool brake_now_stays_inside(const shared_kinematics::Guard& guard, const TrajectoryGenerator& traj) const {
    const std::optional<std::vector<std::array<double, 7>>> poses =
        traj.brake_poses_at(traj.elapsed(), PLAN_CHECK_SPACING_S);
    return poses && !floor_passed_on_(guard, *poses);
  }

 private:
  // Checks the path of a brake from the next point after the brake last checked, and not before where the plan is now.
  std::optional<PlanPass> check_next_brake_(const shared_kinematics::Guard& guard, const TrajectoryGenerator& traj) {
    next_brake_ = std::min(std::max(next_brake_, traj.elapsed()) + PLAN_CHECK_SPACING_S, traj.duration());
    const std::optional<std::vector<std::array<double, 7>>> poses =
        traj.brake_poses_at(next_brake_, PLAN_CHECK_SPACING_S);
    if (!poses) return PlanPass{nearest_floor_(guard, distances_at_(guard, traj.position_at(next_brake_))), false};
    if (std::optional<std::string> side = floor_passed_on_(guard, *poses)) return PlanPass{*side, true};
    return std::nullopt;
  }

  std::optional<std::string> floor_passed_on_(const shared_kinematics::Guard& guard,
                                              const std::vector<std::array<double, 7>>& poses) const {
    for (const std::array<double, 7>& q : poses)
      if (std::optional<std::string> side = floor_passed_(guard, distances_at_(guard, q))) return side;
    return std::nullopt;
  }

  std::optional<PlanPass> check_next_point_(const shared_kinematics::Guard& guard, const TrajectoryGenerator& traj) {
    next_point_ = std::min(next_point_ + PLAN_CHECK_SPACING_S, traj.duration());
    if (std::optional<std::string> side = floor_passed_(guard, distances_at_(guard, traj.position_at(next_point_))))
      return PlanPass{*side, false};
    return std::nullopt;
  }

  std::optional<std::string> floor_passed_(const shared_kinematics::Guard& guard,
                                           const shared_kinematics::Distances& rows) const {
    const double tolerance = guard.params().path_tolerance;
    for (std::size_t k = 0; k < rows.planes.size(); ++k)
      if (rows.planes[k].distance < std::min(rows.planes[k].floor, start_.planes[k].distance) - tolerance)
        return rows.planes[k].side;
    const double clearance = guard.model().self.clearance;
    for (std::size_t k = 0; k < rows.pairs.size(); ++k)
      if (rows.pairs[k].clearance < std::min(clearance, start_.pairs[k].clearance) - tolerance)
        return SELF_COLLISION_KEY;
    return std::nullopt;
  }

  static std::string nearest_floor_(const shared_kinematics::Guard& guard, const shared_kinematics::Distances& rows) {
    std::string nearest = SELF_COLLISION_KEY;
    double least = std::numeric_limits<double>::infinity();
    for (const shared_kinematics::PlaneDistance& row : rows.planes)
      if (row.distance - row.floor < least) {
        least = row.distance - row.floor;
        nearest = row.side;
      }
    for (const shared_kinematics::PairDistance& row : rows.pairs)
      if (row.clearance - guard.model().self.clearance < least) {
        least = row.clearance - guard.model().self.clearance;
        nearest = SELF_COLLISION_KEY;
      }
    return nearest;
  }

  static shared_kinematics::Distances distances_at_(const shared_kinematics::Guard& guard,
                                                    const std::array<double, 7>& q) {
    return guard.distances(Eigen::Map<const Vector7d>(q.data()));
  }

  std::uint64_t plan_ = 0;
  double next_point_ = 0.0;
  double next_brake_ = 0.0;
  shared_kinematics::Distances start_;
};

// The check of a straight joint move against the self pairs of a guard, as the clip checks them: at points close
// enough that a pair falls at most the path tolerance below its floor between two of them, and at least the least
// samples of the guard. The torque loop checks PLAN_CHECK_SAMPLES points at each tick, and moves its reference only on
// the checked part of the move. The speed scale of the box keeps the reference off the sides.
class LineCheck {
 public:
  // Starts the check of the move from `from` to `to`.
  void start(const shared_kinematics::Guard& guard, const Vector7d& from, const Vector7d& to) {
    from_ = from;
    to_ = to;
    const double change = guard.self_rate_bounds().dot((to - from).cwiseAbs());
    samples_ = std::max(guard.params().least_samples,
                        static_cast<int>(std::ceil(change / (2 * guard.params().path_tolerance))));
    checked_ = 0;
    end_ = samples_;
    start_ = guard.distances(from).pairs;
  }

  // Whether points of the move are left to check.
  bool pending() const { return checked_ < end_; }

  // Checks the next points of the move, up to `count`. Returns whether a point brings two links closer than the
  // clearance, or than at the start when that is less; the move then ends at the point before it.
  bool step(const shared_kinematics::Guard& guard, int count) {
    const double clearance = guard.model().self.clearance;
    for (int i = 0; i < count && pending(); ++i) {
      const std::vector<shared_kinematics::PairDistance> pairs = guard.distances(point_(checked_ + 1)).pairs;
      for (std::size_t k = 0; k < pairs.size(); ++k)
        if (pairs[k].clearance < std::min(clearance, start_[k].clearance)) {
          end_ = checked_;
          return true;
        }
      ++checked_;
    }
    return false;
  }

  // The last point of the move that the check let through.
  Vector7d checked_point() const { return point_(checked_); }

  // `next`, a point of the move, kept on its checked part. Any point with no check started.
  Vector7d within_checked(const Vector7d& next) const {
    if (samples_ == 0) return next;
    const Vector7d move = to_ - from_;
    const double squared = move.squaredNorm();
    if (!(squared > 0.0)) return next;
    const double along = (next - from_).dot(move) / squared;
    return along * samples_ > checked_ ? checked_point() : next;
  }

 private:
  Vector7d point_(int i) const { return i >= samples_ ? to_ : Vector7d(from_ + (to_ - from_) * i / samples_); }

  Vector7d from_ = Vector7d::Zero();
  Vector7d to_ = Vector7d::Zero();
  int samples_ = 0;
  int checked_ = 0;
  int end_ = 0;
  std::vector<shared_kinematics::PairDistance> start_;
};

// Near a side the position loop plays its plan at a velocity scale of 1, 1/2, 1/4 and so on, down to this one.
constexpr double LEAST_VELOCITY_SCALE = 1.0 / 64.0;

// A velocity scale for the playing plan, and the side that set it when it is lower than the scale in force.
struct VelocityStep {
  double scale = 1.0;
  std::optional<std::string> slowest;
};

// The velocity scale for the next tick of `traj` near the sides of `box`, in steps of a factor 2 so that the plan
// changes only on a step. It steps down, to LEAST_VELOCITY_SCALE at the least, when a bounded part moves toward a side
// faster than the approach gain times its distance to the floor, and steps up when four times the scale keeps every
// part within that speed.
inline VelocityStep velocity_step_near(const shared_kinematics::Guard& box, const TrajectoryGenerator& traj,
                                       double dt) {
  const double scale = traj.velocity_scale();
  if (!traj.active()) return {scale, std::nullopt};
  const Vector7d q = Eigen::Map<const Vector7d>(traj.position().data());
  const Vector7d v = Eigen::Map<const Vector7d>(traj.velocity().data());
  // The share of its velocity limit at which the plan moves now.
  double share = 0.0;
  for (int j = 0; j < 7; ++j) share = std::max(share, std::abs(v[j]) / traj.velocity_limit()[j]);
  if (!(share > 0.0)) return {scale, std::nullopt};
  const shared_kinematics::ScaleResult now = box.speed_scale(q, v * dt, dt);
  if (now.s < 1.0) {
    double step = scale;
    while (step > LEAST_VELOCITY_SCALE && step > share * now.s) step /= 2.0;
    return step < scale ? VelocityStep{step, now.slowest} : VelocityStep{scale, std::nullopt};
  }
  if (scale < 1.0) {
    const double up = std::min(1.0, 2.0 * scale);
    if (!(box.speed_scale(q, v * (2.0 * up / share) * dt, dt).s < 1.0)) return {up, std::nullopt};
  }
  return {scale, std::nullopt};
}

// Brakes `traj` before `side`. A brake that the planner refuses leaves the arm on a plan past the side, so it raises
// std::runtime_error, which stops the control loop.
template <typename Trajectory>
void brake_before(Trajectory& traj, const std::string& side) {
  if (!traj.brake_to_rest()) throw std::runtime_error("the trajectory planner refused the brake before side " + side);
}

}  // namespace positronic_franka
