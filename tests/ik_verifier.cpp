#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "robot.hpp"

namespace {

using positronic_franka::Vector7d;

Vector7d clamp_to_limits(const Vector7d& q) {
  Vector7d clamped = q;
  for (int i = 0; i < 7; ++i) {
    clamped(i) = std::min(positronic_franka::PANDA_JOINT_UPPER_LIMITS[static_cast<size_t>(i)],
                          std::max(positronic_franka::PANDA_JOINT_LOWER_LIMITS[static_cast<size_t>(i)],
                                   q(i)));
  }
  return clamped;
}

double quaternion_distance_deg(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  Eigen::Quaterniond rel = a.conjugate() * b;
  rel.normalize();
  double angle = 2.0 * std::atan2(rel.vec().norm(), std::abs(rel.w()));
  return angle * 180.0 / M_PI;
}

struct Metrics {
  double max_position_error_mm = 0.0;
  double max_rotation_error_deg = 0.0;
  size_t failures = 0;
  size_t successes = 0;
  size_t attempts = 0;
};

Metrics verify_method(positronic_franka::Robot& robot,
                      const std::vector<Vector7d>& target_configs,
                      const Vector7d& q_seed,
                      bool use_limits_solver,
                      double tol_pos_mm,
                      double tol_rot_deg) {
  Metrics metrics;
  metrics.attempts = target_configs.size();

  auto format_vec = [](const Vector7d& v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << "[";
    for (int i = 0; i < v.size(); ++i) {
      oss << v(i);
      if (i != v.size() - 1)
        oss << ", ";
    }
    oss << "]";
    return oss.str();
  };

  for (const auto& q_target : target_configs) {
    Vector7d pose_vec = robot.forward_kinematics(q_target);
    Eigen::Vector3d t_target = pose_vec.head<3>();
    Eigen::Quaterniond quat_target(pose_vec(3), pose_vec(4), pose_vec(5), pose_vec(6));
    quat_target.normalize();

    Vector7d q_solution;
    try {
      if (use_limits_solver) {
        q_solution = robot.inverse_kinematics_with_limits(pose_vec, q_seed);
      } else {
        q_solution = robot.inverse_kinematics_q0(pose_vec, q_seed);
      }
    } catch (const std::exception& e) {
      ++metrics.failures;
      std::cerr << "[IK] Exception for solver=" << (use_limits_solver ? "limits" : "dls")
                << ": " << e.what() << '\n'
                << "      target_q=" << format_vec(q_target) << '\n'
                << "      seed_q  =" << format_vec(q_seed) << '\n';
      continue;
    }
    Vector7d achieved_pose_vec = robot.forward_kinematics(q_solution);
    Eigen::Vector3d t_achieved = achieved_pose_vec.head<3>();
    Eigen::Quaterniond quat_achieved(achieved_pose_vec(3), achieved_pose_vec(4),
                                     achieved_pose_vec(5), achieved_pose_vec(6));
    quat_achieved.normalize();

    double pos_err_mm = (t_target - t_achieved).norm() * 1000.0;
    double rot_err_deg = quaternion_distance_deg(quat_target, quat_achieved);

    metrics.max_position_error_mm = std::max(metrics.max_position_error_mm, pos_err_mm);
    metrics.max_rotation_error_deg = std::max(metrics.max_rotation_error_deg, rot_err_deg);

    if (pos_err_mm > tol_pos_mm || rot_err_deg > tol_rot_deg) {
      ++metrics.failures;
      std::cerr << std::fixed << std::setprecision(3)
                << "[IK] Pose error exceeds tolerance (" << (use_limits_solver ? "limits" : "dls")
                << ") pos=" << pos_err_mm << " mm (tol=" << tol_pos_mm
                << " mm) rot=" << rot_err_deg << " deg (tol=" << tol_rot_deg << " deg)\n"
                << "      target_q =" << format_vec(q_target) << '\n'
                << "      seed_q   =" << format_vec(q_seed) << '\n'
                << "      solution=" << format_vec(q_solution);
      if (use_limits_solver) {
        try {
          Vector7d dls_sol = robot.inverse_kinematics_q0(pose_vec, q_seed);
          std::cerr << '\n' << "      dls_sol =" << format_vec(dls_sol);
        } catch (...) {
          std::cerr << '\n' << "      dls_sol = <exception>";
        }
      }
      std::cerr << '\n';
      continue;
    }
    ++metrics.successes;
  }

  return metrics;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argc > 4) {
    std::cerr << "Usage: " << argv[0] << " <robot_ip> [samples=100] [seed=0]\n";
    return EXIT_FAILURE;
  }

  const std::string ip = argv[1];
  const size_t samples = (argc >= 3) ? static_cast<size_t>(std::strtoul(argv[2], nullptr, 10)) : 100;
  const unsigned int seed = (argc == 4) ? static_cast<unsigned int>(std::strtoul(argv[3], nullptr, 10))
                                        : 0;

  positronic_franka::Robot robot(ip);
  positronic_franka::State base_state = robot.state();
  Vector7d q_seed = base_state.q;

  std::mt19937 rng(seed ? seed : static_cast<unsigned int>(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  std::normal_distribution<double> noise(0.0, 0.05);  // radian std dev

  std::vector<Vector7d> targets;
  targets.reserve(samples);
  for (size_t i = 0; i < samples; ++i) {
    Vector7d q_sample = q_seed;
    for (int j = 0; j < 7; ++j) {
      q_sample(j) += noise(rng);
    }
    q_sample = clamp_to_limits(q_sample);
    targets.push_back(q_sample);
  }

  constexpr double kTolPosMm = 0.5;
  constexpr double kTolRotDeg = 0.5;

  Metrics dls_metrics = verify_method(robot, targets, q_seed, false, kTolPosMm, kTolRotDeg);
  Metrics limits_metrics = verify_method(robot, targets, q_seed, true, kTolPosMm, kTolRotDeg);

  auto print_metrics = [](const std::string& label, const Metrics& m) {
    std::cout << label << ":\n"
              << "  Attempts: " << m.attempts << "\n"
              << "  Successes: " << m.successes << "\n"
              << "  Failures: " << m.failures << "\n"
              << std::fixed << std::setprecision(3)
              << "  Max position error: " << m.max_position_error_mm << " mm\n"
              << "  Max rotation error: " << m.max_rotation_error_deg << " deg\n";
  };

  print_metrics("Damped IK", dls_metrics);
  print_metrics("Limits IK", limits_metrics);

  const bool success = (dls_metrics.failures == 0 && limits_metrics.failures == 0);
  return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
