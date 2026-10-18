#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "impedance_law.hpp"

namespace positronic_franka {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using SpatialJacobian = Eigen::Matrix<double, 6, 7>;

// FR3 joint position limits (rad), as in the FR3 URDF that the robot reports.
constexpr std::array<double, 7> FR3_JOINT_LOWER_LIMITS = {
    -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
constexpr std::array<double, 7> FR3_JOINT_UPPER_LIMITS = {
    2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};

// The IK damps the move when the smallest singular value of the tool Jacobian is below this value.
// The rotation rows count 1 m per rad.
constexpr double IK_DAMPING_SINGULAR_VALUE = 0.1;
// Each joint moves at most this far from the measured joints per command, rad, so the answer stays on the
// current IK branch.
constexpr double IK_MAX_JOINT_STEP = 0.5;
// Joints that the move pulls towards along the null space of the tool Jacobian.
constexpr std::array<double, 7> IK_POSTURE_JOINTS = {0.0, -0.31, 0.0, -1.65, 0.0, 1.522, 0.0};
// Part of the null-space offset from IK_POSTURE_JOINTS that one command removes, and its weight.
constexpr double IK_POSTURE_PULL_PER_COMMAND = 0.02;
constexpr double IK_POSTURE_WEIGHT = 0.1;

// The tool pose (4x4, base frame) and its zero Jacobian (6x7, linear rows first) at one q.
struct ToolKinematics {
  Eigen::Matrix4d pose;
  SpatialJacobian jacobian;
};

// Position error, then rotation error as an angle-axis vector in the base frame. J dq moves it.
inline Vector6d pose_error(const Eigen::Matrix4d& T_cur, const Eigen::Vector3d& t_tgt,
                           const Eigen::Matrix3d& R_tgt) {
  const Eigen::Matrix3d R_cur = T_cur.block<3, 3>(0, 0);
  const Eigen::AngleAxisd aa(R_cur.transpose() * R_tgt);
  Vector6d e;
  e << T_cur.block<3, 1>(0, 3) - t_tgt, -R_cur * (aa.angle() * aa.axis());
  return e;
}

// Where minimize_on_box holds each joint: free, at its lower or upper bound, or fixed by a zero-width box.
enum class BoxHold { kFree, kLower, kUpper, kFixed };
using BoxHolds = std::array<BoxHold, 7>;

// argmin 0.5 d^T H d + g^T d over the free joints, with every other joint held at its value in d.
inline Vector7d newton_point_on_free_joints(const Matrix7d& H, const Vector7d& g, const Vector7d& d,
                                            const BoxHolds& hold) {
  std::array<int, 7> free_index{};
  int n_free = 0;
  for (int i = 0; i < 7; ++i)
    if (hold[i] == BoxHold::kFree) free_index[n_free++] = i;
  Vector7d p = d;
  if (n_free == 0) return p;
  Eigen::MatrixXd H_ff(n_free, n_free);
  Eigen::VectorXd rhs(n_free);
  for (int a = 0; a < n_free; ++a) {
    const int i = free_index[a];
    rhs(a) = -g(i);
    for (int j = 0; j < 7; ++j)
      if (hold[j] != BoxHold::kFree) rhs(a) -= H(i, j) * d(j);
    for (int b = 0; b < n_free; ++b) H_ff(a, b) = H(i, free_index[b]);
  }
  const Eigen::VectorXd x = H_ff.llt().solve(rhs);
  for (int a = 0; a < n_free; ++a) p(free_index[a]) = x(a);
  return p;
}

// The free joint whose bound the move from d to p meets first, and the part of the move that reaches it.
struct FirstBound {
  int joint = -1;  // -1: the move stays inside the box
  double fraction = 1.0;
};

inline FirstBound first_bound_on_the_move(const Vector7d& d, const Vector7d& p, const Vector7d& lo,
                                          const Vector7d& hi, const BoxHolds& hold) {
  FirstBound first;
  for (int i = 0; i < 7; ++i) {
    if (hold[i] != BoxHold::kFree) continue;
    const double edge = p(i) < lo(i) ? lo(i) : p(i) > hi(i) ? hi(i) : p(i);
    if (edge == p(i)) continue;
    const double fraction = (edge - d(i)) / (p(i) - d(i));
    if (fraction < first.fraction) first = {i, fraction};
  }
  return first;
}

// The held joint whose move into the box lowers the cost most, or -1 when no move does.
inline int joint_to_release(const Vector7d& grad, const BoxHolds& hold) {
  int release = -1;
  double strongest = 1e-12;
  for (int i = 0; i < 7; ++i) {
    const double pull = hold[i] == BoxHold::kLower ? -grad(i) : hold[i] == BoxHold::kUpper ? grad(i) : 0.0;
    if (pull > strongest) {
      strongest = pull;
      release = i;
    }
  }
  return release;
}

// argmin 0.5 d^T H d + g^T d subject to lo <= d <= hi, by a primal active set.
// H must be positive definite, and lo <= 0 <= hi, so that d = 0 is a feasible start.
inline Vector7d minimize_on_box(const Matrix7d& H, const Vector7d& g, const Vector7d& lo, const Vector7d& hi) {
  BoxHolds hold{};
  Vector7d d = Vector7d::Zero();
  for (int i = 0; i < 7; ++i)
    if (lo(i) >= hi(i)) {
      hold[i] = BoxHold::kFixed;
      d(i) = lo(i);
    }
  // Each pass either holds a joint at a bound or frees one, so a few passes per joint are enough.
  for (int pass = 0; pass < 64; ++pass) {
    const Vector7d p = newton_point_on_free_joints(H, g, d, hold);
    const FirstBound first = first_bound_on_the_move(d, p, lo, hi, hold);
    if (first.joint >= 0) {
      d += first.fraction * (p - d);
      const bool at_lower = p(first.joint) < lo(first.joint);
      d(first.joint) = at_lower ? lo(first.joint) : hi(first.joint);
      hold[first.joint] = at_lower ? BoxHold::kLower : BoxHold::kUpper;
      continue;
    }
    d = p;
    const int release = joint_to_release(H * d + g, hold);
    if (release < 0) return d;
    hold[release] = BoxHold::kFree;
  }
  return d;
}

// Joints that put the tool as close as the bounds allow to `target_pose_wxyz` (tx, ty, tz, qw, qx, qy, qz).
// Each joint stays within the FR3 limits and within `max_step` of q0; near a singularity the move q - q0 is damped.
template <typename Kinematics>
Vector7d solve_pose_ik(const Kinematics& kinematics, const Eigen::Ref<const Vector7d>& target_pose_wxyz,
                       const Eigen::Ref<const Vector7d>& q0, double max_step) {
  constexpr int kMaxIterations = 60;
  constexpr double kStepTolerance = 1e-9;  // rad
  // Keeps each Gauss-Newton step finite at an exact singularity; it vanishes at the fixed point.
  constexpr double kLevenberg = 1e-4;

  const Eigen::Vector3d t_tgt = target_pose_wxyz.head<3>();
  const Eigen::Matrix3d R_tgt =
      Eigen::Quaterniond(target_pose_wxyz(3), target_pose_wxyz(4), target_pose_wxyz(5), target_pose_wxyz(6))
          .normalized()
          .toRotationMatrix();

  // Eigenvalues of J^T J ascend: the first is the null space, the second is s_min squared.
  const SpatialJacobian J0 = kinematics(Vector7d(q0)).jacobian;
  const Eigen::SelfAdjointEigenSolver<Matrix7d> eig(J0.transpose() * J0);
  const double s_min = std::sqrt(std::max(0.0, eig.eigenvalues()(1)));
  const Vector7d null_direction = eig.eigenvectors().col(0);
  // Chiaverini's law; mu_max = S0 / sqrt(2) keeps the weak-direction gain s / (s^2 + mu^2) at or below 1 / S0.
  const double S0 = IK_DAMPING_SINGULAR_VALUE;
  const double mu_sq = s_min < S0 ? 0.5 * S0 * S0 * (1.0 - (s_min / S0) * (s_min / S0)) : 0.0;

  const Vector7d posture = Eigen::Map<const Vector7d>(IK_POSTURE_JOINTS.data());
  const double posture_target = -IK_POSTURE_PULL_PER_COMMAND * null_direction.dot(q0 - posture);
  const double w_sq = IK_POSTURE_WEIGHT * IK_POSTURE_WEIGHT;

  Vector7d lower, upper;
  for (int i = 0; i < 7; ++i) {
    lower(i) = std::max(FR3_JOINT_LOWER_LIMITS[i], q0(i) - max_step);
    upper(i) = std::min(FR3_JOINT_UPPER_LIMITS[i], q0(i) + max_step);
  }
  Vector7d q = q0.cwiseMax(lower).cwiseMin(upper);

  for (int it = 0; it < kMaxIterations; ++it) {
    const ToolKinematics k = kinematics(q);
    const SpatialJacobian& J = k.jacobian;
    const Vector6d e = pose_error(k.pose, t_tgt, R_tgt);
    const double posture_residual = null_direction.dot(q - q0) - posture_target;
    const Matrix7d H = J.transpose() * J + (mu_sq + kLevenberg) * Matrix7d::Identity() +
                       w_sq * null_direction * null_direction.transpose();
    const Vector7d g = J.transpose() * e + mu_sq * (q - q0) + w_sq * posture_residual * null_direction;
    const Vector7d d = minimize_on_box(H, g, lower - q, upper - q);
    q = (q + d).cwiseMax(lower).cwiseMin(upper);
    if (d.norm() < kStepTolerance) break;
  }
  return q;
}

}  // namespace positronic_franka
