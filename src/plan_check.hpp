#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <shared_kinematics/shared_kinematics.hpp>

#include "impedance_law.hpp"
#include "trajectory_generator.hpp"
#include "workspace_bounds.hpp"

namespace positronic_franka {

// The position loop checks a plan at points this far apart in time: PLAN_CHECK_SAMPLES of them when the plan starts,
// then one more at each tick, so the check stays ahead of the arm.
constexpr double PLAN_CHECK_SPACING_S = 0.01;
constexpr int PLAN_CHECK_SAMPLES = 16;

// The check of the playing plan against the sides and the self pairs of a guard. The floor of each check is its floor,
// or its value where the check started when that is less, less the path tolerance. A self pair reports
// SELF_COLLISION_KEY.
class PlanCheck {
 public:
  // Starts the check of the plan that `traj` plays now, from where it is, and checks its first points. Returns the
  // first side that a bounded part passes at them.
  std::optional<std::string> start(const shared_kinematics::Guard& guard, const TrajectoryGenerator& traj) {
    plan_ = traj.plan_number();
    next_ = traj.elapsed();
    start_ = rows_at_(guard, traj, next_);
    std::optional<std::string> side;
    for (int i = 0; i < PLAN_CHECK_SAMPLES && !side && pending(traj); ++i) side = step(guard, traj);
    return side;
  }

  // Whether points of the plan that the check started on are left to check.
  bool pending(const TrajectoryGenerator& traj) const {
    return plan_ != 0 && traj.plan_number() == plan_ && next_ < traj.duration();
  }

  // Checks the next point of the plan. Returns the side that a bounded part passes at it.
  std::optional<std::string> step(const shared_kinematics::Guard& guard, const TrajectoryGenerator& traj) {
    next_ = std::min(next_ + PLAN_CHECK_SPACING_S, traj.duration());
    const shared_kinematics::Distances rows = rows_at_(guard, traj, next_);
    const double tolerance = guard.params().path_tolerance;
    for (std::size_t k = 0; k < rows.planes.size(); ++k)
      if (rows.planes[k].distance < std::min(rows.planes[k].floor, start_.planes[k].distance) - tolerance)
        return rows.planes[k].side;
    const double clearance = guard.model().self.clearance;
    for (std::size_t k = 0; k < rows.pairs.size(); ++k)
      if (rows.pairs[k].clearance < std::min(clearance, start_.pairs[k].clearance) - tolerance)
        return std::string(SELF_COLLISION_KEY);
    return std::nullopt;
  }

 private:
  static shared_kinematics::Distances rows_at_(const shared_kinematics::Guard& guard, const TrajectoryGenerator& traj,
                                               double t) {
    const std::array<double, 7> q = traj.position_at(t);
    return guard.distances(Eigen::Map<const Vector7d>(q.data()));
  }

  std::uint64_t plan_ = 0;
  double next_ = 0.0;
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

}  // namespace positronic_franka
