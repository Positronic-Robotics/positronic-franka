#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "fr3_arm.hpp"
#include "impedance_law.hpp"
#include "self_collision.hpp"

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
// Part of the null-space offset from IK_POSTURE_JOINTS that one command removes.
constexpr double IK_POSTURE_PULL_PER_COMMAND = 0.02;
// Below this clearance between two checked links, the IK pushes them apart. It is above
// IK_SELF_COLLISION_CLEARANCE, so the soft push holds the answer clear of the hard floor.
constexpr double IK_SELF_COLLISION_PUSH_CLEARANCE = 0.10;
// Within this distance above its floor at a side, the IK pushes a bounded part away from the side.
constexpr double IK_PLANE_PUSH_BAND = 0.01;

// The damped IK stops when the norm of the pose error is below this value, or after DAMPED_IK_MAX_ITERATIONS steps.
constexpr double DAMPED_IK_TOLERANCE = 1e-4;
constexpr int DAMPED_IK_MAX_ITERATIONS = 150;
// The lambda of the damped pseudo-inverse J^T (J J^T + lambda I)^-1.
constexpr double DAMPED_IK_DAMPING = 0.03;
// The gain of the pull of the joints towards zero in the null space.
constexpr double DAMPED_IK_NULLSPACE_GAIN = 0.002;
// The line search tries this part of the step first, and multiplies it by DAMPED_IK_LINE_SEARCH_BETA per trial.
// It stops after DAMPED_IK_LINE_SEARCH_MAX_STEPS trials, or when the part is below DAMPED_IK_MIN_STEP.
constexpr double DAMPED_IK_LINE_SEARCH_ALPHA = 1.0;
constexpr double DAMPED_IK_LINE_SEARCH_BETA = 0.5;
constexpr int DAMPED_IK_LINE_SEARCH_MAX_STEPS = 20;
constexpr double DAMPED_IK_MIN_STEP = 1e-8;

// The tool pose (4x4, base frame) and its zero Jacobian (6x7, linear rows first) at one q.
struct ToolKinematics {
  Eigen::Matrix4d pose;
  SpatialJacobian jacobian;
};

// The zero Jacobian of a point fixed to the tool, from the tool kinematics. `tool_to_point` is the point in
// tool axes, from the tool origin, in metres. The angular rows are the tool's own.
inline SpatialJacobian point_jacobian(const ToolKinematics& tool, const Eigen::Vector3d& tool_to_point) {
  const Eigen::Vector3d r = tool.pose.block<3, 3>(0, 0) * tool_to_point;
  SpatialJacobian J = tool.jacobian;
  for (int i = 0; i < 7; ++i) J.block<3, 1>(0, i) += J.block<3, 1>(3, i).cross(r);
  return J;
}

// The flange pose in the base frame, O_T_F, from the end-effector pose O_T_EE and the tool offset F_T_EE.
inline Eigen::Matrix4d flange_pose(const Eigen::Matrix4d& O_T_EE, const Eigen::Matrix4d& F_T_EE) {
  return (Eigen::Isometry3d(O_T_EE) * Eigen::Isometry3d(F_T_EE).inverse()).matrix();
}

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

// J^T (J J^T + lambda I)^-1.
inline Eigen::Matrix<double, 7, 6> damped_pinv(const SpatialJacobian& J, double lambda) {
  return J.transpose() * (J * J.transpose() + lambda * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
}

// The projector onto the null space of J: a step through it leaves the tool pose unchanged to first order.
inline Matrix7d nullspace_projector(const SpatialJacobian& J) {
  const Eigen::JacobiSVD<SpatialJacobian> svd(J, Eigen::ComputeFullV);
  const auto row_space = svd.matrixV().leftCols(svd.rank());
  return Matrix7d::Identity() - row_space * row_space.transpose();
}

// Joints that put the tool at `target_pose_wxyz` (tx, ty, tz, qw, qx, qy, qz), from q0, by damped least-squares
// steps with a pull of q towards zero in the null space and a backtracking line search. No joint bounds.
template <typename Kinematics>
Vector7d solve_damped_pose_ik(const Kinematics& kinematics, const Eigen::Ref<const Vector7d>& target_pose_wxyz,
                              const Eigen::Ref<const Vector7d>& q0, double tol = DAMPED_IK_TOLERANCE,
                              int max_iters = DAMPED_IK_MAX_ITERATIONS, double min_step = DAMPED_IK_MIN_STEP,
                              double pinv_reg = DAMPED_IK_DAMPING, double nullspace_gain = DAMPED_IK_NULLSPACE_GAIN,
                              double line_search_alpha = DAMPED_IK_LINE_SEARCH_ALPHA,
                              double line_search_beta = DAMPED_IK_LINE_SEARCH_BETA,
                              int line_search_max_steps = DAMPED_IK_LINE_SEARCH_MAX_STEPS) {
  const Eigen::Vector3d t_tgt = target_pose_wxyz.head<3>();
  const Eigen::Matrix3d R_tgt =
      Eigen::Quaterniond(target_pose_wxyz(3), target_pose_wxyz(4), target_pose_wxyz(5), target_pose_wxyz(6))
          .normalized()
          .toRotationMatrix();

  Vector7d q = q0;
  for (int it = 0; it < max_iters; ++it) {
    const ToolKinematics k = kinematics(q);
    const Vector6d e = pose_error(k.pose, t_tgt, R_tgt);
    const double err_norm = e.norm();
    if (err_norm < tol) break;

    const Eigen::Matrix<double, 7, 6> J_pinv = damped_pinv(k.jacobian, pinv_reg);
    // Not I - J_pinv J: with damping, that matrix moves the tool, and the pull then holds the error above tol.
    const Matrix7d N = nullspace_projector(k.jacobian);
    const Vector7d dq = -J_pinv * e + N * (-nullspace_gain * std::exp(err_norm) * q);

    double step = line_search_alpha;
    double best_err = err_norm;
    for (int ls = 0; ls < line_search_max_steps; ++ls) {
      const Vector7d q_trial = q + step * dq;
      const double err_trial = pose_error(kinematics(q_trial).pose, t_tgt, R_tgt).norm();
      if (err_trial < best_err) {
        best_err = err_trial;
        q = q_trial;
      }
      step *= line_search_beta;
      if (step < min_step) break;
    }
    if (best_err >= err_norm - 1e-9) break;  // no step lowers the error
  }
  return q;
}

// The point of the move from `from` to `to` farthest along it whose path keeps each checked pair at its floor:
// IK_SELF_COLLISION_CLEARANCE, or its own clearance at `from` when that is less.
// Between two checked points a pair's clearance falls at most IK_SELF_COLLISION_PATH_TOLERANCE below its floor.
// The walk is the clip of the shared core on the FR3 self pairs (fr3_self_guard).
inline Vector7d stop_before_self_collision(const Vector7d& from, const Vector7d& to) {
  return fr3_self_guard().clip(from, to).q;
}

// Joints that put the tool as close as the bounds allow to `target_pose_wxyz` (tx, ty, tz, qw, qx, qy, qz).
// Each joint stays within the FR3 limits and within `max_step` of q0; near a singularity the move q - q0 is damped.
// The solve pushes the self pairs and the bounded parts of `guard` away from their floors. Then the move from q0, or
// from its nearest point within the FR3 limits, stops at the clip of `guard`.
template <typename Kinematics>
Vector7d solve_pose_ik(const Kinematics& kinematics, const Eigen::Ref<const Vector7d>& target_pose_wxyz,
                       const Eigen::Ref<const Vector7d>& q0, double max_step, const sk::Guard& guard) {
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
  constexpr double kPostureWeight = 0.1;
  const double w_sq = kPostureWeight * kPostureWeight;
  // 1 cm inside IK_SELF_COLLISION_PUSH_CLEARANCE costs as much as 20 cm of tool error.
  constexpr double kCollisionWeight = 20.0;
  constexpr double kCollisionWeightSq = kCollisionWeight * kCollisionWeight;

  Vector7d lower, upper;
  for (int i = 0; i < 7; ++i) {
    // The step term goes first: std::max and std::min return their first argument when the other is NaN.
    lower(i) = std::max(q0(i) - max_step, FR3_JOINT_LOWER_LIMITS[i]);
    upper(i) = std::min(q0(i) + max_step, FR3_JOINT_UPPER_LIMITS[i]);
  }
  // minimize_on_box needs a box that holds a point. NaN fails the comparison too.
  if (!(lower.array() <= upper.array()).all())
    throw std::invalid_argument("no joints are within max_step of q0 and inside the FR3 limits");
  const Vector7d start = q0.cwiseMax(lower).cwiseMin(upper);

  // One push term: how far a check is inside its push band (negative) and its gradient in the joints.
  struct Push {
    double inside;
    Vector7d gradient;
  };
  // One iterate: its kinematics, its push terms, and the cost that the Gauss-Newton step models.
  struct Iterate {
    Vector7d q;
    ToolKinematics k;
    Vector6d e;
    std::vector<Push> pushes;
    double posture_residual;
    double cost;
  };
  const auto pushes_at = [&](const Vector7d& q) {
    const sk::Distances rows = guard.distances(q);
    std::vector<Push> pushes;
    for (const sk::PairDistance& pair : rows.pairs)
      if (pair.clearance < IK_SELF_COLLISION_PUSH_CLEARANCE)
        pushes.push_back({pair.clearance - IK_SELF_COLLISION_PUSH_CLEARANCE, pair.gradient});
    for (const sk::PlaneDistance& plane : rows.planes)
      if (plane.distance < plane.floor + IK_PLANE_PUSH_BAND)
        pushes.push_back({plane.distance - plane.floor - IK_PLANE_PUSH_BAND, plane.gradient});
    return pushes;
  };
  const auto iterate_at = [&](const Vector7d& q) {
    Iterate x{q, kinematics(q), Vector6d::Zero(), pushes_at(q), 0.0, 0.0};
    x.e = pose_error(x.k.pose, t_tgt, R_tgt);
    x.posture_residual = null_direction.dot(q - q0) - posture_target;
    x.cost = 0.5 * (x.e.squaredNorm() + mu_sq * (q - q0).squaredNorm() +
                    w_sq * x.posture_residual * x.posture_residual);
    for (const Push& push : x.pushes) x.cost += 0.5 * kCollisionWeightSq * push.inside * push.inside;
    return x;
  };
  // A backtracking step lowers the cost by at least kSufficientDecrease of the slope's prediction, in at most
  // kLineSearchSteps trials.
  constexpr int kLineSearchSteps = 10;
  constexpr double kSufficientDecrease = 1e-4;
  struct Descent {
    Vector7d q;
    bool settled;
  };
  // Gauss-Newton from the start. With `backtrack`, each step is halved until the cost falls enough.
  const auto descend = [&](bool backtrack) {
    Iterate at = iterate_at(start);
    for (int it = 0; it < kMaxIterations; ++it) {
      const SpatialJacobian& J = at.k.jacobian;
      Matrix7d H = J.transpose() * J + (mu_sq + kLevenberg) * Matrix7d::Identity() +
                   w_sq * null_direction * null_direction.transpose();
      Vector7d g = J.transpose() * at.e + mu_sq * (at.q - q0) + w_sq * at.posture_residual * null_direction;
      for (const Push& push : at.pushes) {
        H += kCollisionWeightSq * push.gradient * push.gradient.transpose();
        g += kCollisionWeightSq * push.inside * push.gradient;
      }
      const Vector7d d = minimize_on_box(H, g, lower - at.q, upper - at.q);
      if (d.norm() < kStepTolerance) return Descent{(at.q + d).cwiseMax(lower).cwiseMin(upper), true};
      double part = 1.0;
      Iterate next = iterate_at((at.q + d).cwiseMax(lower).cwiseMin(upper));
      const auto too_high = [&] { return backtrack && next.cost > at.cost + kSufficientDecrease * part * g.dot(d); };
      for (int ls = 1; ls < kLineSearchSteps && too_high(); ++ls) {
        part *= 0.5;
        next = iterate_at((at.q + part * d).cwiseMax(lower).cwiseMin(upper));
      }
      if (too_high()) return Descent{at.q, true};  // no step lowers the cost
      at = next;
    }
    return Descent{at.q, false};
  };
  // Full steps can overshoot the push and cycle without settling; the backtracking descent then gives the answer.
  Descent answer = descend(false);
  if (!answer.settled) answer = descend(true);
  return guard.clip(start, answer.q).q;
}

// solve_pose_ik on the FR3 self pairs, with no box.
template <typename Kinematics>
Vector7d solve_pose_ik(const Kinematics& kinematics, const Eigen::Ref<const Vector7d>& target_pose_wxyz,
                       const Eigen::Ref<const Vector7d>& q0, double max_step) {
  return solve_pose_ik(kinematics, target_pose_wxyz, q0, max_step, fr3_self_guard());
}

}  // namespace positronic_franka
