#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <shared_kinematics/shared_kinematics.hpp>

namespace positronic_franka {

// A real desk or wall (kSurface), or a line that an operator chose (kVirtual).
using SideKind = shared_kinematics::SideKind;

// One plane of the workspace box, in the rig frame: a point x is inside when n . x + d >= 0, with n of unit length.
// pad_allowance and rigid_allowance are how far past the side a finger pad and a rigid part may go, in metres; a
// negative allowance is a margin that the part keeps inside the side.
using Side = shared_kinematics::Plane;

// The key under which the robot state counts the targets that the self-collision clip shortened. No side takes it.
inline const std::string SELF_COLLISION_KEY = "self";

// How far a normal or a rotation may be from unit length or orthonormal.
constexpr double WORKSPACE_BOUNDS_UNIT_TOLERANCE = 1e-6;

// The sides of the workspace box and the pose of the arm base in the rig frame. The sides are kept in the order
// of their names, so the hash does not depend on the order in which the caller gives them.
class WorkspaceBounds {
 public:
  explicit WorkspaceBounds(std::vector<Side> sides, const Eigen::Matrix4d& base_pose = Eigen::Matrix4d::Identity())
      : sides_(std::move(sides)), base_pose_(base_pose) {
    std::sort(sides_.begin(), sides_.end(), [](const Side& a, const Side& b) { return a.name < b.name; });
    validate_();
    hash_ = compute_hash_();
  }

  const std::vector<Side>& sides() const { return sides_; }
  // The arm base in the rig frame, T_rig_base.
  const Eigen::Matrix4d& base_pose() const { return base_pose_; }
  // 16 hex digits of the 64-bit FNV-1a hash of the names, the kinds, every number of the sides, and the base pose.
  const std::string& hash() const { return hash_; }

 private:
  void validate_() const {
    if (sides_.empty()) throw std::invalid_argument("workspace bounds need at least one side; pass None for no bounds");
    for (std::size_t i = 0; i < sides_.size(); ++i) {
      const Side& side = sides_[i];
      if (side.name.empty()) throw std::invalid_argument("a side needs a name");
      if (side.name == SELF_COLLISION_KEY)
        throw std::invalid_argument("the side name '" + SELF_COLLISION_KEY + "' is reserved for self-collision");
      if (i > 0 && sides_[i - 1].name == side.name)
        throw std::invalid_argument("two sides have the name '" + side.name + "'");
      if (!side.n.allFinite() || std::abs(side.n.norm() - 1.0) > WORKSPACE_BOUNDS_UNIT_TOLERANCE)
        throw std::invalid_argument("the normal n of side '" + side.name + "' must be finite and of unit length");
      if (!std::isfinite(side.d) || !std::isfinite(side.pad_allowance) || !std::isfinite(side.rigid_allowance))
        throw std::invalid_argument("the offset d and the allowances of side '" + side.name + "' must be finite");
    }
    const Eigen::Matrix3d R = base_pose_.topLeftCorner<3, 3>();
    const bool rigid = base_pose_.allFinite() && base_pose_.row(3) == Eigen::RowVector4d(0, 0, 0, 1) &&
                       (R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <=
                           WORKSPACE_BOUNDS_UNIT_TOLERANCE &&
                       R.determinant() > 0.0;
    if (!rigid) throw std::invalid_argument("base_pose must be a finite rigid transform (4x4, last row 0 0 0 1)");
  }

  std::string compute_hash_() const {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    const auto bytes = [&h](const void* data, std::size_t size) {
      const auto* p = static_cast<const unsigned char*>(data);
      for (std::size_t i = 0; i < size; ++i) h = (h ^ p[i]) * 0x100000001b3ULL;
    };
    const auto number = [&bytes](double v) {
      v += 0.0;  // -0.0 hashes as 0.0
      std::uint64_t bits = 0;
      std::memcpy(&bits, &v, sizeof bits);
      bytes(&bits, sizeof bits);
    };
    for (const Side& side : sides_) {
      const std::uint64_t length = side.name.size();
      bytes(&length, sizeof length);
      bytes(side.name.data(), side.name.size());
      const unsigned char kind = side.kind == SideKind::kSurface ? 0 : 1;
      bytes(&kind, 1);
      for (int i = 0; i < 3; ++i) number(side.n(i));
      number(side.d);
      number(side.pad_allowance);
      number(side.rigid_allowance);
    }
    for (int i = 0; i < 16; ++i) number(base_pose_(i));
    static const char* digits = "0123456789abcdef";
    std::string out(16, '0');
    for (int i = 15; i >= 0; --i, h >>= 4) out[i] = digits[h & 0xf];
    return out;
  }

  std::vector<Side> sides_;
  Eigen::Matrix4d base_pose_;
  std::string hash_;
};

// What the guard did since the last set_workspace_bounds, or since the start of the Robot.
struct GuardReport {
  // Per side, and SELF_COLLISION_KEY for the self-collision clip: how many targets the clip shortened there.
  std::map<std::string, std::uint64_t> bindings;
  // Per side: the deepest that a bounded point went past its floor at that side, from the measured joints, m.
  // A side that no point passed has no entry.
  std::map<std::string, double> deepest;

  void count_binding(const std::string& key) { ++bindings[key]; }
  void record_depth(const std::string& side, double past_floor) {
    if (!(past_floor > 0.0)) return;
    auto [it, added] = deepest.emplace(side, past_floor);
    if (!added) it->second = std::max(it->second, past_floor);
  }
};

}  // namespace positronic_franka
