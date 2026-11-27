#pragma once

#include <shared_kinematics/shared_kinematics.hpp>

#include "fr3_arm.hpp"

// The FR3 with bounded parts: the self-collision capsules of links 2 .. 7, a gripper body, and a pad point where
// the closed pads of a 2F-85 meet, 0.155 m along the flange z axis.
inline shared_kinematics::ArmModel fr3_with_gripper() {
  shared_kinematics::ArmModel model = positronic_franka::fr3_arm_model();
  for (const auto& c : model.self.capsules) model.bodies.push_back({c, shared_kinematics::Role::kRigid, c.link > 2});
  const int flange = positronic_franka::FR3_FLANGE_LINK;
  model.bodies.push_back({{"gripper", flange, {0, 0, 0.01}, {0, 0, 0.11}, 0.045}, shared_kinematics::Role::kRigid});
  model.bodies.push_back({{"pads", flange, {0, 0, 0.155}, {0, 0, 0.155}, 0.0}, shared_kinematics::Role::kPad});
  return model;
}
