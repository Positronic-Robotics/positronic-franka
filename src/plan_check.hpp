#pragma once

#include <algorithm>
#include <array>
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

}  // namespace positronic_franka
