#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <shared_kinematics/shared_kinematics.hpp>

#include "self_collision.hpp"

namespace positronic_franka {

namespace sk = shared_kinematics;

// The link of the FR3 chain that holds link k of the FR3 is k + 1; this one is link8, the flange.
constexpr int FR3_FLANGE_LINK = 9;

// The links of the FR3 URDF that the robot reports: base, link0 .. link7, and link8, the flange.
inline sk::Chain fr3_chain() {
  constexpr double kHalfPi = 1.570796326794897;  // as the URDF writes it
  const auto origin = [](double x, double y, double z, double roll) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(x, y, z);
    t.linear() = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return t;
  };
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  return sk::Chain({
      {"base", -1, Eigen::Isometry3d::Identity(), -1, z},
      {"link0", 0, Eigen::Isometry3d::Identity(), -1, z},
      {"link1", 1, origin(0, 0, 0.333, 0), 0, z},
      {"link2", 2, origin(0, 0, 0, -kHalfPi), 1, z},
      {"link3", 3, origin(0, -0.316, 0, kHalfPi), 2, z},
      {"link4", 4, origin(0.0825, 0, 0, kHalfPi), 3, z},
      {"link5", 5, origin(-0.0825, 0.384, 0, -kHalfPi), 4, z},
      {"link6", 6, origin(0, 0, 0, kHalfPi), 5, z},
      {"link7", 7, origin(0.088, 0, 0, kHalfPi), 6, z},
      {"link8", 8, origin(0, 0, 0.107, 0), -1, z},
  });
}

// The self-collision model of self_collision.hpp: its capsules, its link pairs and IK_SELF_COLLISION_CLEARANCE.
inline sk::SelfCollision fr3_self_collision() {
  sk::SelfCollision self;
  for (const Capsule& c : fr3_self_collision_capsules()) {
    const Eigen::Vector3d half = c.axis * (0.5 * c.length);
    self.capsules.push_back({"link" + std::to_string(c.link), c.link + 1, c.center - half, c.center + half, c.radius});
  }
  for (const auto& [la, lb] : FR3_SELF_COLLISION_PAIRS) {
    sk::SelfPair pair{"link" + std::to_string(la) + "-link" + std::to_string(lb), {}, {}};
    for (int i = 0; i < static_cast<int>(self.capsules.size()); ++i) {
      if (self.capsules[i].link == la + 1) pair.a.push_back(i);
      if (self.capsules[i].link == lb + 1) pair.b.push_back(i);
    }
    self.pairs.push_back(pair);
  }
  self.clearance = IK_SELF_COLLISION_CLEARANCE;
  return self;
}

// The FR3 with its self-collision model and no bounded part. A box needs a model with bounded parts.
inline sk::ArmModel fr3_arm_model() { return {fr3_chain(), {}, fr3_self_collision(), Eigen::Isometry3d::Identity()}; }

// The walk of stop_before_self_collision: at most IK_SELF_COLLISION_PATH_TOLERANCE below a floor between two samples.
inline sk::GuardParams fr3_guard_params() {
  sk::GuardParams params;
  params.path_tolerance = IK_SELF_COLLISION_PATH_TOLERANCE;
  return params;
}

// The guard of the FR3 self pairs, with no box.
inline const sk::Guard& fr3_self_guard() {
  static const sk::Guard guard(fr3_arm_model(), {}, fr3_guard_params());
  return guard;
}

}  // namespace positronic_franka
