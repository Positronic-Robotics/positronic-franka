// Checks the pose IK in pose_ik.hpp on the FR3 kinematics. Needs only Eigen, not libfranka:
//   g++ -std=c++17 -I src $(pkg-config --cflags eigen3) tests/test_pose_ik.cpp -o test_pose_ik
//   ./test_pose_ik

#include <array>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "pose_ik.hpp"

namespace {

using positronic_franka::FR3_JOINT_LOWER_LIMITS;
using positronic_franka::FR3_JOINT_UPPER_LIMITS;
using positronic_franka::IK_DAMPING_SINGULAR_VALUE;
using positronic_franka::IK_MAX_JOINT_STEP_AFTER_PAUSE;
using positronic_franka::IK_PAUSE;
using positronic_franka::IK_POSTURE_JOINTS;
using positronic_franka::IK_POSTURE_PULL_PER_COMMAND;
using positronic_franka::Matrix7d;
using positronic_franka::max_joint_step;
using positronic_franka::minimize_on_box;
using positronic_franka::pose_error;
using positronic_franka::solve_pose_ik;
using positronic_franka::SpatialJacobian;
using positronic_franka::ToolKinematics;
using positronic_franka::Vector6d;
using positronic_franka::Vector7d;

struct Failure {
  std::string what;
};

void check(bool ok, const std::string& what) {
  if (!ok) throw Failure{what};
}

constexpr double kPi = 3.14159265358979323846;

// FR3 kinematics from Franka's modified DH parameters, with the Franka Hand frame (the robot's kEndEffector).
ToolKinematics fr3_tool(const Vector7d& q) {
  const std::array<double, 8> a = {0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0};
  const std::array<double, 8> d = {0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107};
  const std::array<double, 8> alpha = {0, -kPi / 2, kPi / 2, kPi / 2, -kPi / 2, kPi / 2, kPi / 2, 0};
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  std::array<Eigen::Vector3d, 7> axis, origin;
  for (int i = 0; i < 8; ++i) {
    T = T * Eigen::AngleAxisd(alpha[i], Eigen::Vector3d::UnitX()) * Eigen::Translation3d(a[i], 0, 0) *
        Eigen::AngleAxisd(i < 7 ? q(i) : 0.0, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(0, 0, d[i]);
    if (i < 7) {
      axis[i] = T.linear().col(2);
      origin[i] = T.translation();
    }
  }
  T = T * Eigen::AngleAxisd(-kPi / 4, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(0, 0, 0.1034);
  ToolKinematics k{T.matrix(), SpatialJacobian::Zero()};
  for (int i = 0; i < 7; ++i) {
    k.jacobian.block<3, 1>(0, i) = axis[i].cross(T.translation() - origin[i]);
    k.jacobian.block<3, 1>(3, i) = axis[i];
  }
  return k;
}

Vector7d pose_wxyz(const Eigen::Matrix4d& T) {
  const Eigen::Quaterniond quat(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
  Vector7d pose;
  pose << T.block<3, 1>(0, 3), quat.w(), quat.x(), quat.y(), quat.z();
  return pose;
}

// Tool miss of q against the target, in mm and deg.
std::pair<double, double> miss(const Vector7d& q, const Vector7d& target) {
  const Eigen::Matrix3d R =
      Eigen::Quaterniond(target(3), target(4), target(5), target(6)).normalized().toRotationMatrix();
  const Vector6d e = pose_error(fr3_tool(q).pose, target.head<3>(), R);
  return {e.head<3>().norm() * 1e3, e.tail<3>().norm() * 180.0 / kPi};
}

Vector7d posture() { return Eigen::Map<const Vector7d>(IK_POSTURE_JOINTS.data()); }

void minimize_on_box_meets_the_optimality_conditions() {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int trial = 0; trial < 500; ++trial) {
    Matrix7d M;
    for (int i = 0; i < 49; ++i) M(i) = u(rng);
    const Matrix7d H = M * M.transpose() + 1e-3 * Matrix7d::Identity();
    Vector7d g, lo, hi;
    for (int i = 0; i < 7; ++i) {
      g(i) = 3.0 * u(rng);
      lo(i) = -std::abs(u(rng));
      hi(i) = trial % 10 == 0 ? 0.0 : std::abs(u(rng));
    }
    const Vector7d d = minimize_on_box(H, g, lo, hi);
    const Vector7d grad = H * d + g;
    for (int i = 0; i < 7; ++i) {
      check(d(i) >= lo(i) - 1e-12 && d(i) <= hi(i) + 1e-12, "the solution is inside the box");
      const bool at_lower = d(i) <= lo(i) + 1e-12, at_upper = d(i) >= hi(i) - 1e-12;
      const bool stationary = (at_lower && grad(i) >= -1e-8) || (at_upper && grad(i) <= 1e-8) ||
                              std::abs(grad(i)) < 1e-8;
      check(stationary, "no joint can lower the cost by moving inside the box");
    }
  }
}

void minimize_on_box_without_an_active_bound_is_the_newton_step() {
  Matrix7d H = Matrix7d::Identity();
  H(0, 1) = H(1, 0) = 0.3;
  const Vector7d g = Vector7d::LinSpaced(-0.5, 0.5);
  const Vector7d d = minimize_on_box(H, g, Vector7d::Constant(-10.0), Vector7d::Constant(10.0));
  check((d + H.llt().solve(g)).norm() < 1e-12, "a wide box returns -H^-1 g");
}

void max_joint_step_is_the_speed_bound_times_the_command_interval() {
  check(std::abs(max_joint_step(1.0 / 15.0) - 0.2) < 1e-12, "15 Hz gives 0.2 rad");
  check(std::abs(max_joint_step(0.2) - 0.6) < 1e-12, "5 Hz gives 0.6 rad");
  check(std::abs(max_joint_step(IK_PAUSE) - 3.0 * IK_PAUSE) < 1e-12, "the bound holds up to the pause");
}

void max_joint_step_after_a_pause_or_on_the_first_command_is_the_pause_step() {
  check(max_joint_step(IK_PAUSE + 1e-6) == IK_MAX_JOINT_STEP_AFTER_PAUSE, "a pause gives the pause step");
  check(max_joint_step(3.0) == IK_MAX_JOINT_STEP_AFTER_PAUSE, "recording 14's 3 s gap gives the pause step");
  check(max_joint_step(std::nullopt) == IK_MAX_JOINT_STEP_AFTER_PAUSE, "the first command gives the pause step");
}

void a_nearby_pose_away_from_a_singularity_is_reached_within_0_1_mm_and_0_1_deg() {
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-0.15, 0.15);
  const Vector7d q0 = posture();
  for (int trial = 0; trial < 300; ++trial) {
    Vector7d q_target = q0;
    for (int i = 0; i < 7; ++i) q_target(i) += u(rng);
    const Vector7d target = pose_wxyz(fr3_tool(q_target).pose);
    const auto [mm, deg] = miss(solve_pose_ik(fr3_tool, target, q0, 0.2), target);
    check(mm < 0.1 && deg < 0.1, "miss " + std::to_string(mm) + " mm, " + std::to_string(deg) + " deg");
  }
}

Vector7d limit(const std::array<double, 7>& limits) { return Eigen::Map<const Vector7d>(limits.data()); }

void every_joint_stays_within_the_step_bound_and_the_fr3_limits() {
  std::mt19937 rng(13);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const Vector7d lower = limit(FR3_JOINT_LOWER_LIMITS), upper = limit(FR3_JOINT_UPPER_LIMITS);
  for (int trial = 0; trial < 300; ++trial) {
    Vector7d q0, q_target;
    for (int i = 0; i < 7; ++i) {
      // Starts on a limit in one trial out of three, to load the limit bound.
      const double t = trial % 3 == 0 ? (u(rng) > 0 ? 1.0 : 0.0) : 0.5 + 0.5 * u(rng);
      q0(i) = lower(i) + t * (upper(i) - lower(i));
      q_target(i) = q0(i) + u(rng);
    }
    const double step = trial % 2 == 0 ? 0.2 : 0.05;
    const Vector7d q = solve_pose_ik(fr3_tool, pose_wxyz(fr3_tool(q_target).pose), q0, step);
    check(((q - q0).cwiseAbs().array() <= step + 1e-12).all(), "a joint stepped past the bound");
    check((q.array() >= lower.array()).all() && (q.array() <= upper.array()).all(), "a joint left its limits");
  }
}

// A measured FR3 pose just before a wrist-singularity spin: joint 6 near pi, s_min 0.041.
Vector7d near_wrist_singularity() {
  Vector7d q;
  q << 0.810, 0.0319, 0.060, -2.5503, 1.0741, 3.1396, -0.9533;
  return q;
}

void near_a_singularity_the_weak_direction_gain_is_at_most_one_over_the_threshold() {
  const Vector7d q0 = near_wrist_singularity();
  const ToolKinematics k = fr3_tool(q0);
  const SpatialJacobian& J = k.jacobian;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> left(J * J.transpose());
  const double s = std::sqrt(left.eigenvalues()(0));
  const Vector6d weak_twist = left.eigenvectors().col(0);
  const Vector7d weak_joints = J.transpose() * weak_twist / s;
  check(s < 0.5 * IK_DAMPING_SINGULAR_VALUE, "the start is deep inside the damped region");

  // A small tool twist along the weak left singular vector, applied to the tool pose.
  const double eps = 0.002;
  const Vector6d twist = eps * weak_twist;
  Eigen::Matrix4d T = k.pose;
  T.block<3, 1>(0, 3) += twist.head<3>();
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(twist.tail<3>().norm(), twist.tail<3>().normalized()).toRotationMatrix() *
                        k.pose.block<3, 3>(0, 0);
  const Vector7d q = solve_pose_ik(fr3_tool, pose_wxyz(T), q0, 0.2);

  const double weak_move = std::abs(weak_joints.dot(q - q0));
  const double S0 = IK_DAMPING_SINGULAR_VALUE;
  const double mu_sq = 0.5 * S0 * S0 * (1.0 - (s / S0) * (s / S0));
  check(weak_move <= eps / S0, "the weak-direction move exceeds the change over the threshold");
  check(std::abs(weak_move - eps * s / (s * s + mu_sq)) < 0.1 * weak_move, "the move follows the damping law");
  check(weak_move < 0.5 * eps / s, "the move is less than half the undamped move");
}

Vector7d null_direction(const Vector7d& q) {
  const SpatialJacobian J = fr3_tool(q).jacobian;
  return Eigen::SelfAdjointEigenSolver<Matrix7d>(J.transpose() * J).eigenvectors().col(0);
}

void the_move_pulls_the_posture_along_the_null_space_and_keeps_the_tool() {
  const Vector7d home = posture();
  const Vector7d q0 = home + 0.5 * null_direction(home);
  const Vector7d n0 = null_direction(q0);
  const Vector7d target = pose_wxyz(fr3_tool(q0).pose);

  const Vector7d q = solve_pose_ik(fr3_tool, target, q0, 0.2);
  const double expected = -IK_POSTURE_PULL_PER_COMMAND * n0.dot(q0 - home);
  check(std::abs(n0.dot(q - q0) - expected) < 0.05 * std::abs(expected), "the null-space pull is 2 % of the offset");
  const auto [mm, deg] = miss(q, target);
  check(mm < 0.1 && deg < 0.1, "the pull moved the tool");
}

}  // namespace

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
      {"minimize_on_box_meets_the_optimality_conditions", minimize_on_box_meets_the_optimality_conditions},
      {"minimize_on_box_without_an_active_bound_is_the_newton_step",
       minimize_on_box_without_an_active_bound_is_the_newton_step},
      {"max_joint_step_is_the_speed_bound_times_the_command_interval",
       max_joint_step_is_the_speed_bound_times_the_command_interval},
      {"max_joint_step_after_a_pause_or_on_the_first_command_is_the_pause_step",
       max_joint_step_after_a_pause_or_on_the_first_command_is_the_pause_step},
      {"a_nearby_pose_away_from_a_singularity_is_reached_within_0_1_mm_and_0_1_deg",
       a_nearby_pose_away_from_a_singularity_is_reached_within_0_1_mm_and_0_1_deg},
      {"every_joint_stays_within_the_step_bound_and_the_fr3_limits",
       every_joint_stays_within_the_step_bound_and_the_fr3_limits},
      {"near_a_singularity_the_weak_direction_gain_is_at_most_one_over_the_threshold",
       near_a_singularity_the_weak_direction_gain_is_at_most_one_over_the_threshold},
      {"the_move_pulls_the_posture_along_the_null_space_and_keeps_the_tool",
       the_move_pulls_the_posture_along_the_null_space_and_keeps_the_tool},
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
  return failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
