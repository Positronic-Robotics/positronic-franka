#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include <shared_kinematics/shared_kinematics.hpp>

#include "fr3_arm.hpp"
#include "workspace_bounds.hpp"

namespace positronic_franka {

// The core guards of one arm for one box. A guard that would check nothing is empty.
struct ArmGuards {
  // inverse_kinematics_with_limits: the self pairs and the box.
  std::shared_ptr<const sk::Guard> ik;
  // set_target_joints: the box, and the self pairs when the self-collision clip is on.
  std::shared_ptr<const sk::Guard> target;
  // The control loops: the box alone.
  std::shared_ptr<const sk::Guard> box;
};

// The guards of `model` with `bounds`. The arm base takes the base pose of the bounds. Raises std::invalid_argument
// for a model whose joints are not the 7 of the FR3, and for bounds on a model with no bounded part.
inline ArmGuards make_arm_guards(const sk::ArmModel& model, const std::optional<WorkspaceBounds>& bounds,
                                 bool clip_self_collision) {
  if (model.chain.joint_count() != 7) throw std::invalid_argument("the arm model must have the 7 joints of the FR3");
  sk::ArmModel placed = model;
  std::vector<sk::Plane> planes;
  if (bounds) {
    if (std::none_of(model.bodies.begin(), model.bodies.end(), [](const sk::Body& b) { return b.in_planes; }))
      throw std::invalid_argument("workspace bounds need an arm model with bounded parts");
    placed.base_pose.matrix() = bounds->base_pose();
    planes = bounds->sides();
  }
  sk::ArmModel no_self = placed;
  no_self.self.pairs.clear();
  ArmGuards guards;
  guards.ik = std::make_shared<const sk::Guard>(placed, planes, fr3_guard_params());
  if (bounds) guards.box = std::make_shared<const sk::Guard>(no_self, planes, fr3_guard_params());
  guards.target = clip_self_collision ? guards.ik : guards.box;
  return guards;
}

}  // namespace positronic_franka
