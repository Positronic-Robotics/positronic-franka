// Checks the pose IK in pose_ik.hpp and its self-collision model in self_collision.hpp on the FR3 flange
// kinematics. Needs only Eigen, not libfranka:
//   g++ -std=c++17 -I src $(pkg-config --cflags eigen3) tests/test_pose_ik.cpp -o test_pose_ik
//   ./test_pose_ik

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pose_ik.hpp"

namespace {

using positronic_franka::closest_points_of_segments;
using positronic_franka::flange_pose;
using positronic_franka::fr3_clearance_rate_bounds;
using positronic_franka::fr3_link_frames;
using positronic_franka::FR3_JOINT_LOWER_LIMITS;
using positronic_franka::FR3_JOINT_UPPER_LIMITS;
using positronic_franka::FR3_SELF_COLLISION_PAIRS;
using positronic_franka::IK_DAMPING_SINGULAR_VALUE;
using positronic_franka::IK_MAX_JOINT_STEP;
using positronic_franka::IK_POSTURE_JOINTS;
using positronic_franka::IK_POSTURE_PULL_PER_COMMAND;
using positronic_franka::IK_SELF_COLLISION_CLEARANCE;
using positronic_franka::IK_SELF_COLLISION_PATH_TOLERANCE;
using positronic_franka::IK_SELF_COLLISION_PUSH_CLEARANCE;
using positronic_franka::Matrix7d;
using positronic_franka::minimize_on_box;
using positronic_franka::nullspace_projector;
using positronic_franka::point_jacobian;
using positronic_franka::pose_error;
using positronic_franka::self_collision_clearances;
using positronic_franka::SelfCollisionClearance;
using positronic_franka::solve_damped_pose_ik;
using positronic_franka::solve_pose_ik;
using positronic_franka::stop_before_self_collision;
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

// The FR3 flange (link8, the robot's kFlange) and the zero Jacobian there, from Franka's modified DH parameters.
ToolKinematics fr3_flange(const Vector7d& q) {
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
  const Vector6d e = pose_error(fr3_flange(q).pose, target.head<3>(), R);
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

void a_nearby_pose_away_from_a_singularity_is_reached_within_0_1_mm_and_0_1_deg() {
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-0.15, 0.15);
  const Vector7d q0 = posture();
  for (int trial = 0; trial < 300; ++trial) {
    Vector7d q_target = q0;
    for (int i = 0; i < 7; ++i) q_target(i) += u(rng);
    const Vector7d target = pose_wxyz(fr3_flange(q_target).pose);
    const auto [mm, deg] = miss(solve_pose_ik(fr3_flange, target, q0, IK_MAX_JOINT_STEP), target);
    check(mm < 0.1 && deg < 0.1, "miss " + std::to_string(mm) + " mm, " + std::to_string(deg) + " deg");
  }
}

// Joints at q0 plus 0.05 rad of normal noise on each, inside the FR3 limits: the targets of ik_verifier.
std::vector<Vector7d> joints_near(const Vector7d& q0, unsigned int seed, int count) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.05);
  std::vector<Vector7d> joints;
  for (int n = 0; n < count; ++n) {
    Vector7d q = q0;
    for (int i = 0; i < 7; ++i) {
      q(i) = std::clamp(q(i) + noise(rng), FR3_JOINT_LOWER_LIMITS[static_cast<size_t>(i)],
                        FR3_JOINT_UPPER_LIMITS[static_cast<size_t>(i)]);
    }
    joints.push_back(q);
  }
  return joints;
}

void the_damped_ik_reaches_a_nearby_pose_within_its_tolerance() {
  const Vector7d droid_home = (Vector7d() << 0.0, -0.628, 0.0, -2.513, 0.0, 1.885, 0.0).finished();
  for (const Vector7d& q0 : {posture(), droid_home}) {
    for (const Vector7d& q_target : joints_near(q0, 17, 100)) {
      const Vector7d target = pose_wxyz(fr3_flange(q_target).pose);
      const auto [mm, deg] = miss(solve_damped_pose_ik(fr3_flange, target, q0), target);
      check(mm < 0.1 && deg < 0.01, "miss " + std::to_string(mm) + " mm, " + std::to_string(deg) + " deg");
    }
  }
}

void the_null_space_projector_keeps_the_tool_and_the_one_redundant_direction() {
  for (const Vector7d& q : joints_near(posture(), 19, 50)) {
    const SpatialJacobian J = fr3_flange(q).jacobian;
    const Matrix7d N = nullspace_projector(J);
    check((J * N).norm() < 1e-12, "a step through the projector moves the tool");
    check((N * N - N).norm() < 1e-12, "the projector is not idempotent");
    check(std::abs(N.trace() - 1.0) < 1e-9, "the projector frees " + std::to_string(N.trace()) + " directions, not 1");
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
    const double step = trial % 2 == 0 ? IK_MAX_JOINT_STEP : 0.05;
    const Vector7d q = solve_pose_ik(fr3_flange, pose_wxyz(fr3_flange(q_target).pose), q0, step);
    check(((q - q0).cwiseAbs().array() <= step + 1e-12).all(), "a joint stepped past the bound");
    check((q.array() >= lower.array()).all() && (q.array() <= upper.array()).all(), "a joint left its limits");
  }
}

bool refuses(const Vector7d& q0, double step) {
  try {
    solve_pose_ik(fr3_flange, pose_wxyz(fr3_flange(posture()).pose), q0, step);
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}

bool refuses_step(double step) { return refuses(posture(), step); }

void a_negative_or_nan_step_bound_is_refused() {
  check(refuses_step(-0.1), "a negative step was accepted");
  check(refuses_step(std::nan("")), "a NaN step was accepted");
}

void a_start_farther_than_the_step_outside_a_limit_is_refused() {
  Vector7d q0 = posture();
  q0(0) = FR3_JOINT_UPPER_LIMITS[0] + IK_MAX_JOINT_STEP + 0.1;
  check(refuses(q0, IK_MAX_JOINT_STEP), "a start with no joints in reach was accepted");
  q0(0) = FR3_JOINT_UPPER_LIMITS[0] + IK_MAX_JOINT_STEP - 0.1;
  check(!refuses(q0, IK_MAX_JOINT_STEP), "a start with joints in reach was refused");
  q0(0) = std::nan("");
  check(refuses(q0, IK_MAX_JOINT_STEP), "a NaN start was accepted");
}

void an_infinite_step_bound_leaves_only_the_fr3_limits() {
  check(!refuses_step(INFINITY), "an infinite step was refused");
  const Vector7d q0 = posture();
  const Vector7d q_target = q0 + Vector7d::Constant(0.8);
  const Vector7d q = solve_pose_ik(fr3_flange, pose_wxyz(fr3_flange(q_target).pose), q0, INFINITY);
  check((q - q0).cwiseAbs().maxCoeff() > IK_MAX_JOINT_STEP, "a joint was held to the default step");
  check((q.array() >= limit(FR3_JOINT_LOWER_LIMITS).array()).all() &&
            (q.array() <= limit(FR3_JOINT_UPPER_LIMITS).array()).all(),
        "a joint left its limits");
}

void a_zero_step_bound_holds_every_joint() {
  check(!refuses_step(0.0), "a zero step was refused");
  const Vector7d q0 = posture();
  const Vector7d target = pose_wxyz(fr3_flange(q0 + Vector7d::Constant(0.1)).pose);
  check((solve_pose_ik(fr3_flange, target, q0, 0.0) - q0).norm() == 0.0, "a joint moved with a zero step");
}

// A measured FR3 pose just before a wrist-singularity spin: joint 6 near pi, s_min 0.043 at the flange.
Vector7d near_wrist_singularity() {
  Vector7d q;
  q << 0.810, 0.0319, 0.060, -2.5503, 1.0741, 3.1396, -0.9533;
  return q;
}

// The tool pose moved by `twist` (linear, then angular), both in the base frame.
Eigen::Matrix4d twisted(const Eigen::Matrix4d& pose, const Vector6d& twist) {
  Eigen::Matrix4d T = pose;
  T.block<3, 1>(0, 3) += twist.head<3>();
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(twist.tail<3>().norm(), twist.tail<3>().normalized()).toRotationMatrix() *
                        pose.block<3, 3>(0, 0);
  return T;
}

void near_a_singularity_the_weak_direction_gain_is_at_most_one_over_the_threshold() {
  const Vector7d q0 = near_wrist_singularity();
  const ToolKinematics k = fr3_flange(q0);
  const SpatialJacobian& J = k.jacobian;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> left(J * J.transpose());
  const double s = std::sqrt(left.eigenvalues()(0));
  const Vector6d weak_twist = left.eigenvectors().col(0);
  const Vector7d weak_joints = J.transpose() * weak_twist / s;
  check(s < 0.5 * IK_DAMPING_SINGULAR_VALUE, "the start is deep inside the damped region");

  // A small tool twist along the weak left singular vector.
  const double eps = 0.002;
  const Vector7d q = solve_pose_ik(fr3_flange, pose_wxyz(twisted(k.pose, eps * weak_twist)), q0, IK_MAX_JOINT_STEP);

  const double weak_move = std::abs(weak_joints.dot(q - q0));
  const double S0 = IK_DAMPING_SINGULAR_VALUE;
  const double mu_sq = 0.5 * S0 * S0 * (1.0 - (s / S0) * (s / S0));
  check(weak_move <= eps / S0, "the weak-direction move exceeds the change over the threshold");
  check(std::abs(weak_move - eps * s / (s * s + mu_sq)) < 0.1 * weak_move, "the move follows the damping law");
  check(weak_move < 0.5 * eps / s, "the move is less than half the undamped move");
}

void near_a_singularity_the_damping_keeps_a_wrist_command_inside_the_step_bound() {
  const Vector7d q0 = near_wrist_singularity();
  const ToolKinematics k = fr3_flange(q0);
  const SpatialJacobian& J = k.jacobian;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> left(J * J.transpose());
  const double s = std::sqrt(left.eigenvalues()(0));
  const Vector6d weak_twist = left.eigenvectors().col(0);
  const Vector7d weak_joints = J.transpose() * weak_twist / s;

  // A 1.7 deg command along the weak direction: the exact answer moves the wrist eps / s, 0.70 rad.
  const double eps = 0.03;
  check(eps / s > IK_MAX_JOINT_STEP, "the exact answer needs more than the step bound");
  const Vector7d q = solve_pose_ik(fr3_flange, pose_wxyz(twisted(k.pose, eps * weak_twist)), q0, IK_MAX_JOINT_STEP);
  check((q - q0).cwiseAbs().maxCoeff() < IK_MAX_JOINT_STEP - 0.05, "the step bound is not what holds the move");
  check(std::abs(weak_joints.dot(q - q0)) <= eps / IK_DAMPING_SINGULAR_VALUE,
        "the weak-direction move exceeds the change over the threshold");
}

Vector7d null_direction(const Vector7d& q) {
  const SpatialJacobian J = fr3_flange(q).jacobian;
  return Eigen::SelfAdjointEigenSolver<Matrix7d>(J.transpose() * J).eigenvectors().col(0);
}

void the_move_pulls_the_posture_along_the_null_space_and_keeps_the_tool() {
  const Vector7d home = posture();
  const Vector7d q0 = home + 0.5 * null_direction(home);
  const Vector7d n0 = null_direction(q0);
  const Vector7d target = pose_wxyz(fr3_flange(q0).pose);

  const Vector7d q = solve_pose_ik(fr3_flange, target, q0, IK_MAX_JOINT_STEP);
  const double expected = -IK_POSTURE_PULL_PER_COMMAND * n0.dot(q0 - home);
  check(std::abs(n0.dot(q - q0) - expected) < 0.05 * std::abs(expected), "the null-space pull is 2 % of the offset");
  const auto [mm, deg] = miss(q, target);
  check(mm < 0.1 && deg < 0.1, "the pull moved the tool");
}

// F_T_EE as Desk sets it for the Franka Hand: 103.4 mm along the flange axis and -45 deg about it.
Eigen::Matrix4d franka_hand_from_flange() {
  return (Eigen::Isometry3d(Eigen::AngleAxisd(-kPi / 4, Eigen::Vector3d::UnitZ())) *
          Eigen::Translation3d(0, 0, 0.1034))
      .matrix();
}

void the_flange_pose_takes_the_tool_offset_off_the_end_effector_pose() {
  std::mt19937 rng(17);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int trial = 0; trial < 100; ++trial) {
    Vector7d q = posture();
    for (int i = 0; i < 7; ++i) q(i) += u(rng);
    const Eigen::Matrix4d O_T_F = fr3_flange(q).pose;
    Eigen::Matrix4d F_T_EE = franka_hand_from_flange();
    if (trial % 2) {
      const Eigen::Vector3d offset(0.05 * u(rng), 0.05 * u(rng), 0.2 * u(rng));
      const Eigen::Vector3d axis = Eigen::Vector3d(u(rng), u(rng), u(rng)).normalized();
      F_T_EE = (Eigen::Translation3d(offset) * Eigen::AngleAxisd(kPi * u(rng), axis)).matrix();
    }
    const Eigen::Matrix4d O_T_EE = O_T_F * F_T_EE;
    check((flange_pose(O_T_EE, F_T_EE) - O_T_F).cwiseAbs().maxCoeff() < 1e-12,
          "the flange pose is not O_T_EE F_T_EE^-1");
  }
}

// The point's own velocity, by central differences of its position, against the shifted flange Jacobian.
void the_point_jacobian_is_the_derivative_of_the_point_position() {
  const Eigen::Vector3d point(0.012, -0.03, 0.155);
  const auto position = [&](const Vector7d& q) {
    const Eigen::Matrix4d T = fr3_flange(q).pose;
    return Eigen::Vector3d(T.block<3, 3>(0, 0) * point + T.block<3, 1>(0, 3));
  };
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int trial = 0; trial < 20; ++trial) {
    Vector7d q = posture();
    for (int i = 0; i < 7; ++i) q(i) += u(rng);
    const SpatialJacobian J = point_jacobian(fr3_flange(q), point);
    constexpr double h = 1e-6;
    for (int i = 0; i < 7; ++i) {
      Vector7d up = q, down = q;
      up(i) += h;
      down(i) -= h;
      const Eigen::Vector3d numeric = (position(up) - position(down)) / (2 * h);
      check((J.block<3, 1>(0, i) - numeric).cwiseAbs().maxCoeff() < 1e-7, "the linear rows miss the point velocity");
    }
    check(J.bottomRows<3>() == fr3_flange(q).jacobian.bottomRows<3>(), "the angular rows are not the flange's");
  }
}

void the_point_jacobian_at_the_flange_origin_is_the_flange_jacobian() {
  const ToolKinematics flange = fr3_flange(posture());
  check(point_jacobian(flange, Eigen::Vector3d::Zero()) == flange.jacobian, "a zero offset changes the Jacobian");
}


void the_link_frames_end_at_the_fr3_flange() {
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int trial = 0; trial < 50; ++trial) {
    Vector7d q = posture();
    for (int i = 0; i < 7; ++i) q(i) += u(rng);
    check((fr3_link_frames(q)[8].matrix() - fr3_flange(q).pose).cwiseAbs().maxCoeff() < 1e-12,
          "link 8 is not the flange");
  }
}

void the_closest_points_of_two_segments_are_on_their_common_normal_or_at_an_end() {
  const Eigen::Vector3d o(0, 0, 0), x(1, 0, 0), z(0, 0, 1);
  const auto [a, b] = closest_points_of_segments(-x, x, Eigen::Vector3d(0, -1, 1), Eigen::Vector3d(0, 1, 1));
  check((a - o).norm() < 1e-12 && (b - z).norm() < 1e-12, "crossed segments miss their common normal");
  const auto [c, d] = closest_points_of_segments(o, x, Eigen::Vector3d(2, 1, 0), Eigen::Vector3d(2, -1, 0));
  check((c - x).norm() < 1e-12 && (d - 2 * x).norm() < 1e-12, "the closest point is not the end of the segment");
  const auto [e, f] = closest_points_of_segments(o, x, z, z + x);
  check(std::abs((e - f).norm() - 1.0) < 1e-12, "parallel segments are not 1 m apart");
}

// Central differences of each pair's clearance, against its gradient.
void the_clearance_gradient_is_the_derivative_of_the_clearance() {
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int trial = 0; trial < 50; ++trial) {
    Vector7d q = posture();
    for (int i = 0; i < 7; ++i) q(i) += 1.5 * u(rng);
    const auto pairs = self_collision_clearances(q);
    constexpr double h = 1e-6;
    for (int i = 0; i < 7; ++i) {
      Vector7d up = q, down = q;
      up(i) += h;
      down(i) -= h;
      const auto above = self_collision_clearances(up), below = self_collision_clearances(down);
      for (std::size_t k = 0; k < pairs.size(); ++k)
        check(std::abs(pairs[k].gradient(i) - (above[k].clearance - below[k].clearance) / (2 * h)) < 1e-6,
              "the gradient misses the clearance derivative");
    }
  }
}

// At random joints over the FR3 range, no joint changes the clearance of a checked pair faster than its bound.
void no_joint_changes_a_clearance_faster_than_its_rate_bound() {
  std::mt19937 rng(13);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const Vector7d bound = fr3_clearance_rate_bounds();
  for (int trial = 0; trial < 2000; ++trial) {
    Vector7d q;
    for (int i = 0; i < 7; ++i)
      q(i) = FR3_JOINT_LOWER_LIMITS[i] + u(rng) * (FR3_JOINT_UPPER_LIMITS[i] - FR3_JOINT_LOWER_LIMITS[i]);
    for (const SelfCollisionClearance& c : self_collision_clearances(q))
      check((c.gradient.cwiseAbs().array() <= bound.array() + 1e-12).all(),
            "a joint changes a clearance faster than its bound");
  }
}

// The index of the checked pair of links `a` and `b` in FR3_SELF_COLLISION_PAIRS.
std::size_t pair_of(int a, int b) {
  const auto& pairs = FR3_SELF_COLLISION_PAIRS;
  return static_cast<std::size_t>(std::find(pairs.begin(), pairs.end(), std::make_pair(a, b)) - pairs.begin());
}

using PairClearances = std::array<double, FR3_SELF_COLLISION_PAIRS.size()>;

PairClearances pair_clearances(const Vector7d& q) {
  PairClearances out;
  const auto pairs = self_collision_clearances(q);
  for (std::size_t k = 0; k < out.size(); ++k) out[k] = pairs[k].clearance;
  return out;
}

// The least clearance of each checked pair at 2000 points of the straight joint move from `from` to `to`.
PairClearances least_pair_clearances_on_the_move(const Vector7d& from, const Vector7d& to) {
  PairClearances least = pair_clearances(from);
  for (int i = 1; i <= 2000; ++i) {
    const PairClearances at = pair_clearances(from + i / 2000.0 * (to - from));
    for (std::size_t k = 0; k < least.size(); ++k) least[k] = std::min(least[k], at[k]);
  }
  return least;
}

double clearance(const Vector7d& q) {
  const PairClearances pairs = pair_clearances(q);
  return *std::min_element(pairs.begin(), pairs.end());
}

// The least clearance at 2000 points of the straight joint move from `from` to `to`.
double least_clearance_on_the_move(const Vector7d& from, const Vector7d& to) {
  double least = clearance(from);
  for (int i = 1; i <= 2000; ++i) least = std::min(least, clearance(from + i / 2000.0 * (to - from)));
  return least;
}

// A long move whose clearance falls to 0.084 m between its points 8/16 and 9/16, which both keep 0.09 m.
// The guard stops the move before the dip, and plays the clear rest of the move from 9/16 to its end.
void a_move_that_dips_between_two_of_sixteen_points_stops_before_the_dip() {
  Vector7d from, to;
  from << -2.025573, 1.055818, 2.246552, -2.790485, 2.673202, 2.095729, -1.071398;
  to << 1.167132, 0.990198, -2.123749, -2.129653, 0.255810, 1.355959, 2.292034;
  for (int i = 0; i <= 16; ++i)
    check(clearance(from + i / 16.0 * (to - from)) >= IK_SELF_COLLISION_CLEARANCE, "a sixteenth point is too close");
  check(least_clearance_on_the_move(from, to) < IK_SELF_COLLISION_CLEARANCE - 0.005, "the move does not dip");
  const Vector7d stop = stop_before_self_collision(from, to);
  check(least_clearance_on_the_move(from, stop) >= IK_SELF_COLLISION_CLEARANCE - IK_SELF_COLLISION_PATH_TOLERANCE,
        "the move goes into the dip");
  check((stop - from).norm() > 0.5 * (to - from).norm(), "the move stops far before the dip");
  const Vector7d after_the_dip = from + 9.0 / 16 * (to - from);
  check(least_clearance_on_the_move(after_the_dip, to) >= IK_SELF_COLLISION_CLEARANCE, "the rest is not clear");
  check(stop_before_self_collision(after_the_dip, to) == to, "the clear rest of the move stops short");
}

// An FR3 posture of a recorded episode: the shoulder back and the elbow folded, link 5 5.1 cm from link 2.
// The self-collision reflex stopped the arm here again and again.
Vector7d shoulder_back_elbow_folded() {
  Vector7d q;
  q << 0.98877418, -1.15661502, -0.96312511, -2.93868756, -0.990777671, 2.05664039, 1.14565432;
  return q;
}

// The flange target of that episode at that posture, 30 cm lower.
Vector7d flange_target_below_the_folded_elbow() {
  Vector7d target;
  target << 0.202441812, -0.005892748, 0.048565254, 0.000218116, 0.993636106, -0.065788185, -0.091428419;
  return target;
}

// Joints a part `s` of the way from the posture to the folded elbow, at 0.123 m clearance for s = 0.84.
Vector7d towards_the_folded_elbow(double s) { return posture() + s * (shoulder_back_elbow_folded() - posture()); }

void the_move_to_a_pose_whose_joints_are_too_close_keeps_the_self_collision_clearance() {
  const Vector7d q0 = towards_the_folded_elbow(0.84);
  const Vector7d target = pose_wxyz(fr3_flange(shoulder_back_elbow_folded()).pose);
  check(clearance(q0) > IK_SELF_COLLISION_PUSH_CLEARANCE, "the start is not clear");
  const Vector7d q = solve_pose_ik(fr3_flange, target, q0, IK_MAX_JOINT_STEP);
  check(least_clearance_on_the_move(q0, q) >= IK_SELF_COLLISION_CLEARANCE - 1e-9, "the move goes inside the clearance");
  check(miss(q, target).first < miss(q0, target).first, "the answer is not nearer the target than the start");
}

void a_pose_whose_joints_are_just_outside_the_push_clearance_is_reached_within_0_1_mm_and_0_1_deg() {
  const Vector7d q_target = towards_the_folded_elbow(0.86);
  const double c = clearance(q_target);
  check(c > IK_SELF_COLLISION_PUSH_CLEARANCE && c < IK_SELF_COLLISION_PUSH_CLEARANCE + 0.02,
        "the target joints are not just outside the push clearance");
  const Vector7d target = pose_wxyz(fr3_flange(q_target).pose);
  const auto [mm, deg] = miss(solve_pose_ik(fr3_flange, target, towards_the_folded_elbow(0.84), IK_MAX_JOINT_STEP),
                              target);
  check(mm < 0.1 && deg < 0.1, "a pose outside the push clearance is not reached");
}

// From the folded elbow, a move that takes link 5 away from link 2 and link 7 towards link 2.
Vector7d link_7_towards_link_2() {
  Vector7d q;
  q << 1.384, -1.477, -0.791, -3.032, -0.499, 1.563, 1.542;
  return q;
}

// Link 5 starts 0.051 m from link 2 and ends 0.078 m from it. Link 7 starts 0.155 m from link 2 and ends 0.073 m
// from it. No pair comes closer than link 5 starts.
void a_move_that_escapes_one_close_pair_does_not_take_another_pair_below_the_floor() {
  const Vector7d from = shoulder_back_elbow_folded(), to = link_7_towards_link_2();
  const std::size_t close = pair_of(2, 5), other = pair_of(2, 7);
  const PairClearances start = pair_clearances(from);
  check(start[close] < IK_SELF_COLLISION_CLEARANCE, "link 5 does not start inside the clearance");
  check(start[other] > IK_SELF_COLLISION_PUSH_CLEARANCE, "link 7 does not start clear");
  check(pair_clearances(to)[close] > start[close], "the move does not take link 5 away from link 2");
  check(least_clearance_on_the_move(from, to) >= start[close], "a pair comes closer than link 5 starts");
  check(least_pair_clearances_on_the_move(from, to)[other] < IK_SELF_COLLISION_CLEARANCE - 0.01,
        "link 7 does not come closer than the floor");

  const Vector7d stop = stop_before_self_collision(from, to);
  const PairClearances kept = least_pair_clearances_on_the_move(from, stop);
  for (std::size_t k = 0; k < kept.size(); ++k)
    check(kept[k] >= std::min(IK_SELF_COLLISION_CLEARANCE, start[k]) - IK_SELF_COLLISION_PATH_TOLERANCE,
          "pair " + std::to_string(k) + " goes below its floor");
  check(pair_clearances(stop)[close] > start[close], "the move does not take link 5 away from link 2 at all");
}

// A pair inside the clearance at the start keeps its own start clearance as its floor. A move that takes link 5
// away from link 2 plays to its end at 0.087 m. The reverse move, which brings link 5 closer, stops.
void a_pair_that_starts_inside_the_clearance_keeps_its_start_clearance() {
  const Vector7d from = shoulder_back_elbow_folded();
  Vector7d out;
  out << 1.173, -1.186, -1.034, -2.834, -0.818, 2.141, 1.153;
  const std::size_t close = pair_of(2, 5);
  const double start = pair_clearances(from)[close];
  const PairClearances on_the_way_out = least_pair_clearances_on_the_move(from, out);
  check(pair_clearances(out)[close] < IK_SELF_COLLISION_CLEARANCE, "link 5 ends outside the clearance");
  check(on_the_way_out[close] >= start, "link 5 comes closer than it starts");
  for (std::size_t k = 0; k < on_the_way_out.size(); ++k)
    check(k == close || on_the_way_out[k] > IK_SELF_COLLISION_CLEARANCE, "another pair comes inside the clearance");
  check(stop_before_self_collision(from, out) == out, "the move out stops short");

  const Vector7d in = from - (out - from);
  check(pair_clearances(in)[close] < start, "the reverse move does not bring link 5 closer");
  const Vector7d stop = stop_before_self_collision(from, in);
  check(least_pair_clearances_on_the_move(from, stop)[close] >= start - IK_SELF_COLLISION_PATH_TOLERANCE,
        "the reverse move brings link 5 closer than it starts");
}

// From a start inside the clearance, each command moves the links apart, and none brings them closer.
// The target is 20 cm in front of the base; the clearance holds the flange about 2 cm from it.
void a_start_inside_the_clearance_moves_out_and_reaches_down_to_the_target() {
  const Vector7d target = flange_target_below_the_folded_elbow();
  Vector7d q = shoulder_back_elbow_folded();
  check(clearance(q) < IK_SELF_COLLISION_CLEARANCE, "the start is not inside the clearance");
  for (int command = 0; command < 30; ++command) {
    const Vector7d next = solve_pose_ik(fr3_flange, target, q, IK_MAX_JOINT_STEP);
    const double least = std::min(IK_SELF_COLLISION_CLEARANCE, clearance(q));
    check(least_clearance_on_the_move(q, next) >= least - 1e-9, "a move brings the links closer than its start");
    q = next;
  }
  check(clearance(q) >= IK_SELF_COLLISION_CLEARANCE, "the arm stays inside the clearance");
  check(miss(q, target).first < 25.0, "the flange stays more than 2.5 cm from the target");
}

// The same commands from starts up to 0.03 rad away in each joint: each start reaches the target as closely.
void from_starts_around_the_folded_elbow_the_arm_moves_out_and_reaches_down_to_the_target() {
  const Vector7d target = flange_target_below_the_folded_elbow();
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-0.03, 0.03);
  for (int trial = 0; trial < 12; ++trial) {
    Vector7d q = shoulder_back_elbow_folded();
    for (int i = 0; i < 7; ++i) q(i) += u(rng);
    for (int command = 0; command < 30; ++command) q = solve_pose_ik(fr3_flange, target, q, IK_MAX_JOINT_STEP);
    check(miss(q, target).first < 25.0, "a start stays more than 2.5 cm from the target");
  }
}

// An FR3 posture of a recorded episode: joint 4 near its lower limit, link 5 0.103 m from link 2.
Vector7d elbow_at_the_push_clearance() {
  Vector7d q;
  q << -1.2091, -0.82226, 0.673168, -2.9921, 0.232892, 2.59085, -0.345722;
  return q;
}

// The flange target of that episode at that posture, 4 cm away, where link 5 must come closer to link 2.
Vector7d flange_target_past_the_push_clearance() {
  Vector7d target;
  target << 0.219762598048, -0.191817523923, 0.334007425486, 0.214049269342, 0.947713690896, -0.231792174796,
      0.047896326427;
  return target;
}

// One command again and again. The arm moves half the way to each answer, as a software impedance reference does.
// Full Gauss-Newton steps there overshoot the push and cycle between two postures.
void a_static_command_at_the_push_clearance_does_not_alternate_between_two_postures() {
  const Vector7d target = flange_target_past_the_push_clearance();
  Vector7d q = elbow_at_the_push_clearance();
  const double c = clearance(q);
  check(c > IK_SELF_COLLISION_PUSH_CLEARANCE && c < IK_SELF_COLLISION_PUSH_CLEARANCE + 0.01,
        "the start is not just outside the push clearance");
  std::vector<Vector7d> answers;
  for (int command = 0; command < 30; ++command) {
    answers.push_back(solve_pose_ik(fr3_flange, target, q, IK_MAX_JOINT_STEP));
    q += 0.5 * (answers.back() - q);
  }
  for (std::size_t k = 2; k < answers.size(); ++k) {
    const bool back = (answers[k] - answers[k - 2]).cwiseAbs().maxCoeff() < 0.01;
    const bool away = (answers[k - 1] - answers[k - 2]).cwiseAbs().maxCoeff() > 0.05;
    check(!(back && away), "answer " + std::to_string(k) + " goes back to the posture of two commands before");
  }
  check((answers.back() - answers[answers.size() - 2]).cwiseAbs().maxCoeff() < 1e-3, "the answers do not settle");
}

}  // namespace

int main() {
  const std::vector<std::pair<const char*, std::function<void()>>> tests = {
      {"minimize_on_box_meets_the_optimality_conditions", minimize_on_box_meets_the_optimality_conditions},
      {"minimize_on_box_without_an_active_bound_is_the_newton_step",
       minimize_on_box_without_an_active_bound_is_the_newton_step},
      {"a_nearby_pose_away_from_a_singularity_is_reached_within_0_1_mm_and_0_1_deg",
       a_nearby_pose_away_from_a_singularity_is_reached_within_0_1_mm_and_0_1_deg},
      {"the_damped_ik_reaches_a_nearby_pose_within_its_tolerance",
       the_damped_ik_reaches_a_nearby_pose_within_its_tolerance},
      {"the_null_space_projector_keeps_the_tool_and_the_one_redundant_direction",
       the_null_space_projector_keeps_the_tool_and_the_one_redundant_direction},
      {"every_joint_stays_within_the_step_bound_and_the_fr3_limits",
       every_joint_stays_within_the_step_bound_and_the_fr3_limits},
      {"a_negative_or_nan_step_bound_is_refused", a_negative_or_nan_step_bound_is_refused},
      {"a_zero_step_bound_holds_every_joint", a_zero_step_bound_holds_every_joint},
      {"a_start_farther_than_the_step_outside_a_limit_is_refused",
       a_start_farther_than_the_step_outside_a_limit_is_refused},
      {"an_infinite_step_bound_leaves_only_the_fr3_limits", an_infinite_step_bound_leaves_only_the_fr3_limits},
      {"near_a_singularity_the_weak_direction_gain_is_at_most_one_over_the_threshold",
       near_a_singularity_the_weak_direction_gain_is_at_most_one_over_the_threshold},
      {"near_a_singularity_the_damping_keeps_a_wrist_command_inside_the_step_bound",
       near_a_singularity_the_damping_keeps_a_wrist_command_inside_the_step_bound},
      {"the_move_pulls_the_posture_along_the_null_space_and_keeps_the_tool",
       the_move_pulls_the_posture_along_the_null_space_and_keeps_the_tool},
      {"the_flange_pose_takes_the_tool_offset_off_the_end_effector_pose",
       the_flange_pose_takes_the_tool_offset_off_the_end_effector_pose},
      {"the_point_jacobian_is_the_derivative_of_the_point_position",
       the_point_jacobian_is_the_derivative_of_the_point_position},
      {"the_point_jacobian_at_the_flange_origin_is_the_flange_jacobian",
       the_point_jacobian_at_the_flange_origin_is_the_flange_jacobian},
      {"the_link_frames_end_at_the_fr3_flange", the_link_frames_end_at_the_fr3_flange},
      {"the_closest_points_of_two_segments_are_on_their_common_normal_or_at_an_end",
       the_closest_points_of_two_segments_are_on_their_common_normal_or_at_an_end},
      {"the_clearance_gradient_is_the_derivative_of_the_clearance",
       the_clearance_gradient_is_the_derivative_of_the_clearance},
      {"no_joint_changes_a_clearance_faster_than_its_rate_bound", no_joint_changes_a_clearance_faster_than_its_rate_bound},
      {"a_move_that_dips_between_two_of_sixteen_points_stops_before_the_dip",
       a_move_that_dips_between_two_of_sixteen_points_stops_before_the_dip},
      {"the_move_to_a_pose_whose_joints_are_too_close_keeps_the_self_collision_clearance",
       the_move_to_a_pose_whose_joints_are_too_close_keeps_the_self_collision_clearance},
      {"a_pose_whose_joints_are_just_outside_the_push_clearance_is_reached_within_0_1_mm_and_0_1_deg",
       a_pose_whose_joints_are_just_outside_the_push_clearance_is_reached_within_0_1_mm_and_0_1_deg},
      {"a_move_that_escapes_one_close_pair_does_not_take_another_pair_below_the_floor",
       a_move_that_escapes_one_close_pair_does_not_take_another_pair_below_the_floor},
      {"a_pair_that_starts_inside_the_clearance_keeps_its_start_clearance",
       a_pair_that_starts_inside_the_clearance_keeps_its_start_clearance},
      {"a_start_inside_the_clearance_moves_out_and_reaches_down_to_the_target",
       a_start_inside_the_clearance_moves_out_and_reaches_down_to_the_target},
      {"from_starts_around_the_folded_elbow_the_arm_moves_out_and_reaches_down_to_the_target",
       from_starts_around_the_folded_elbow_the_arm_moves_out_and_reaches_down_to_the_target},
      {"a_static_command_at_the_push_clearance_does_not_alternate_between_two_postures",
       a_static_command_at_the_push_clearance_does_not_alternate_between_two_postures},
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
