// Checks the workspace box and the guard report in workspace_bounds.hpp, and the guards of arm_guards.hpp. Needs
// Eigen and the headers of the shared kinematics core. The workspace-bounds job of .github/workflows/tests.yaml
// builds it.

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "arm_guards.hpp"
#include "workspace_bounds.hpp"

namespace {

using positronic_franka::GuardReport;
using positronic_franka::Side;
using positronic_franka::SideKind;
using positronic_franka::WorkspaceBounds;

struct Failure {
  std::string what;
};

void check(bool ok, const std::string& what) {
  if (!ok) throw Failure{what};
}

// A desk 2 cm below the base and a front wall 0.8 m ahead of it.
std::vector<Side> desk_and_front() {
  return {
      {"desk", SideKind::kSurface, Eigen::Vector3d::UnitZ(), 0.02, 0.006, -0.005},
      {"front", SideKind::kSurface, -Eigen::Vector3d::UnitX(), 0.8, 0.0, -0.005},
  };
}

bool refused(const std::function<void()>& make) {
  try {
    make();
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}

void the_sides_are_kept_in_the_order_of_their_names() {
  std::vector<Side> sides = desk_and_front();
  std::swap(sides[0], sides[1]);
  const WorkspaceBounds box(sides);
  check(box.sides()[0].name == "desk" && box.sides()[1].name == "front", "the sides are not in the order of names");
  check(box.hash() == WorkspaceBounds(desk_and_front()).hash(), "the hash depends on the order of the sides");
}

void the_hash_of_a_known_box_is_stable() {
  const std::string hash = WorkspaceBounds(desk_and_front()).hash();
  check(hash.size() == 16 && hash.find_first_not_of("0123456789abcdef") == std::string::npos,
        "the hash is not 16 hex digits: " + hash);
  check(hash == "b18324065e351f39", "the hash of the known box changed: " + hash);
}

void the_hash_changes_with_every_name_kind_and_number() {
  std::vector<std::function<void(std::vector<Side>&, Eigen::Matrix4d&)>> changes = {
      [](auto& s, auto&) { s[0].name = "table"; },
      [](auto& s, auto&) { s[0].kind = SideKind::kVirtual; },
      [](auto& s, auto&) { s[0].n = Eigen::Vector3d(0, std::sin(0.01), std::cos(0.01)); },
      [](auto& s, auto&) { s[0].d += 1e-4; },
      [](auto& s, auto&) { s[0].pad_allowance += 1e-4; },
      [](auto& s, auto&) { s[0].rigid_allowance += 1e-4; },
      [](auto&, auto& pose) { pose(2, 3) += 1e-4; },
      [](auto& s, auto&) { s.pop_back(); },
  };
  std::set<std::string> hashes = {WorkspaceBounds(desk_and_front()).hash()};
  for (const auto& change : changes) {
    std::vector<Side> sides = desk_and_front();
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    change(sides, pose);
    hashes.insert(WorkspaceBounds(sides, pose).hash());
  }
  check(hashes.size() == changes.size() + 1, "a change keeps the hash");
}

void a_negative_zero_hashes_as_zero() {
  std::vector<Side> sides = desk_and_front();
  sides[1].pad_allowance = -0.0;
  check(WorkspaceBounds(sides).hash() == WorkspaceBounds(desk_and_front()).hash(), "-0.0 and 0.0 hash apart");
}

void a_bad_box_is_refused() {
  const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<std::string, std::function<void(std::vector<Side>&)>>> bad_sides = {
      {"no side", [](auto& s) { s.clear(); }},
      {"no name", [](auto& s) { s[0].name = ""; }},
      {"the name self", [](auto& s) { s[0].name = "self"; }},
      {"two sides of one name", [](auto& s) { s[1].name = s[0].name; }},
      {"a normal not of unit length", [](auto& s) { s[0].n *= 1.01; }},
      {"a NaN normal", [nan](auto& s) { s[0].n(0) = nan; }},
      {"a NaN offset", [nan](auto& s) { s[0].d = nan; }},
      {"an infinite pad allowance", [inf](auto& s) { s[0].pad_allowance = inf; }},
      {"an infinite rigid allowance", [inf](auto& s) { s[0].rigid_allowance = -inf; }},
  };
  for (const auto& [what, change] : bad_sides) {
    std::vector<Side> sides = desk_and_front();
    change(sides);
    check(refused([&] { WorkspaceBounds box(sides); }), "a box with " + what + " is taken");
  }
  Eigen::Matrix4d scaled = Eigen::Matrix4d::Identity(), reflected = scaled, projective = scaled, nan_pose = scaled;
  scaled(0, 0) = 1.01;
  reflected(2, 2) = -1.0;
  projective(3, 0) = 0.1;
  nan_pose(0, 3) = nan;
  for (const Eigen::Matrix4d& pose : {scaled, reflected, projective, nan_pose})
    check(refused([&] { WorkspaceBounds box(desk_and_front(), pose); }), "a base pose that is not rigid is taken");
}

void a_turned_and_moved_base_pose_is_taken() {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()));
  pose.translation() << 0.3, -0.305, 0.3;
  const WorkspaceBounds box(desk_and_front(), pose.matrix());
  check(box.base_pose().isApprox(pose.matrix(), 0.0), "the base pose is not kept");
}

void the_report_counts_bindings_and_keeps_the_deepest_pass() {
  GuardReport report;
  report.count_binding("desk");
  report.count_binding("desk");
  report.count_binding("self");
  check(report.bindings.at("desk") == 2 && report.bindings.at("self") == 1 && report.bindings.size() == 2,
        "the bindings are not counted per key");
  report.record_depth("desk", 0.002);
  report.record_depth("desk", 0.001);
  report.record_depth("front", 0.0);
  report.record_depth("front", -0.01);
  report.record_depth("left", std::numeric_limits<double>::quiet_NaN());
  check(report.deepest.size() == 1 && report.deepest.at("desk") == 0.002,
        "the report keeps more than the deepest pass of each side");
}

shared_kinematics::ArmModel fr3_with_a_gripper_body() {
  shared_kinematics::ArmModel model = positronic_franka::fr3_arm_model();
  model.bodies.push_back({{"gripper", positronic_franka::FR3_FLANGE_LINK, {0, 0, 0.01}, {0, 0, 0.11}, 0.045}});
  return model;
}

void with_no_bounds_and_no_clip_only_the_ik_has_a_guard_and_it_checks_the_self_pairs() {
  const auto guards = positronic_franka::make_arm_guards(fr3_with_a_gripper_body(), std::nullopt, false);
  check(!guards.target && !guards.box, "a guard checks joint targets or the fast loop with nothing configured");
  check(guards.ik->planes().empty() && guards.ik->model().self.pairs.size() == 8, "the IK guard is not the self pairs");
}

void the_self_collision_clip_checks_joint_targets_against_the_self_pairs() {
  const auto guards = positronic_franka::make_arm_guards(fr3_with_a_gripper_body(), std::nullopt, true);
  check(guards.target && guards.target->planes().empty() && guards.target->model().self.pairs.size() == 8 &&
            !guards.box,
        "the joint target guard does not check the self pairs alone");
}

void bounds_go_to_every_guard_and_only_the_ik_and_the_clip_keep_the_self_pairs() {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 3) = 0.3;
  const WorkspaceBounds bounds(desk_and_front(), pose);
  const auto without_clip = positronic_franka::make_arm_guards(fr3_with_a_gripper_body(), bounds, false);
  check(without_clip.box->model().self.pairs.empty() && without_clip.target->model().self.pairs.empty(),
        "the box guard or the joint target guard checks self pairs without the clip");
  check(without_clip.ik->planes().size() == 2 && without_clip.box->planes().size() == 2 &&
            without_clip.target->planes().size() == 2,
        "a guard has no box");
  check(without_clip.box->model().base_pose.matrix() == pose, "the arm base is not at the base pose of the bounds");
  const auto with_clip = positronic_franka::make_arm_guards(fr3_with_a_gripper_body(), bounds, true);
  check(with_clip.target->model().self.pairs.size() == 8 && with_clip.target->planes().size() == 2,
        "the clip does not check the self pairs and the box");
}

void bounds_on_a_model_with_no_bounded_part_or_a_model_of_other_joints_are_refused() {
  const WorkspaceBounds bounds(desk_and_front());
  check(refused([&] { positronic_franka::make_arm_guards(positronic_franka::fr3_arm_model(), bounds, false); }),
        "bounds on a model with no bounded part are taken");
  shared_kinematics::ArmModel six = fr3_with_a_gripper_body();
  std::vector<shared_kinematics::Link> links = six.chain.links();
  links[8].joint = -1;
  six.chain = shared_kinematics::Chain(links);
  check(refused([&] { positronic_franka::make_arm_guards(six, std::nullopt, false); }), "a 6 joint model is taken");
}

}  // namespace

int main() {
  const std::vector<std::pair<std::string, std::function<void()>>> tests = {
      {"the_sides_are_kept_in_the_order_of_their_names", the_sides_are_kept_in_the_order_of_their_names},
      {"the_hash_of_a_known_box_is_stable", the_hash_of_a_known_box_is_stable},
      {"the_hash_changes_with_every_name_kind_and_number", the_hash_changes_with_every_name_kind_and_number},
      {"a_negative_zero_hashes_as_zero", a_negative_zero_hashes_as_zero},
      {"a_bad_box_is_refused", a_bad_box_is_refused},
      {"a_turned_and_moved_base_pose_is_taken", a_turned_and_moved_base_pose_is_taken},
      {"the_report_counts_bindings_and_keeps_the_deepest_pass", the_report_counts_bindings_and_keeps_the_deepest_pass},
      {"with_no_bounds_and_no_clip_only_the_ik_has_a_guard_and_it_checks_the_self_pairs",
       with_no_bounds_and_no_clip_only_the_ik_has_a_guard_and_it_checks_the_self_pairs},
      {"the_self_collision_clip_checks_joint_targets_against_the_self_pairs",
       the_self_collision_clip_checks_joint_targets_against_the_self_pairs},
      {"bounds_go_to_every_guard_and_only_the_ik_and_the_clip_keep_the_self_pairs",
       bounds_go_to_every_guard_and_only_the_ik_and_the_clip_keep_the_self_pairs},
      {"bounds_on_a_model_with_no_bounded_part_or_a_model_of_other_joints_are_refused",
       bounds_on_a_model_with_no_bounded_part_or_a_model_of_other_joints_are_refused},
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
