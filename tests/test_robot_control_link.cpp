// Checks how Robot in robot.hpp handles a control call stuck in libfranka, on the fake libfranka in
// tests/fake_franka. Needs Eigen and the vendored Ruckig, built as for test_trajectory_generator:
//   g++ -std=c++20 -pthread -I tests/fake_franka -I src -I external/ruckig/include $(pkg-config --cflags eigen3)
//       tests/test_robot_control_link.cpp <ruckig objects> -o test_robot_control_link
//   ./test_robot_control_link

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "robot.hpp"

namespace {

using positronic_franka::GoalStatus;
using positronic_franka::InternalImpedance;
using positronic_franka::Robot;
using positronic_franka::Side;
using positronic_franka::SideKind;
using positronic_franka::SoftwareImpedance;
using positronic_franka::Vector7d;
using positronic_franka::WorkspaceBounds;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

struct Failure {
  std::string what;
};

void check(bool ok, const std::string& what) {
  if (!ok) throw Failure{what};
}

constexpr double TIMEOUT_S = 0.1;
constexpr milliseconds TIMEOUT{100};
constexpr milliseconds OUTER_LIMIT{3000};

franka::fake::Link& link() { return franka::fake::link(); }

Robot make_robot(double timeout_s = TIMEOUT_S) {
  link().reset();
  return Robot("fake", franka::RealtimeConfig::kIgnore, 1.0, InternalImpedance{}, timeout_s);
}

bool wait_until(const std::function<bool()>& done, milliseconds limit = OUTER_LIMIT) {
  const auto give_up = Clock::now() + limit;
  while (Clock::now() < give_up) {
    if (done()) return true;
    std::this_thread::sleep_for(milliseconds(5));
  }
  return done();
}

Vector7d current_q(Robot& robot) { return robot.state().q; }

Vector7d offset_on_joint_1(const Vector7d& q, double rad) {
  Vector7d target = q;
  target(0) += rad;
  return target;
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

// The state can lag the goal by one tick, so wait for the arm to get there.
bool arm_comes_to(Robot& robot, const Vector7d& q) {
  return wait_until([&] { return (robot.state().q - q).cwiseAbs().maxCoeff() < 1e-9; });
}

void the_default_control_link_timeout_is_10_s() {
  check(positronic_franka::DEFAULT_CONTROL_LINK_TIMEOUT_S == 10.0, "the default timeout is 10 s");
}

void a_bad_control_link_timeout_is_refused() {
  for (const double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
    bool refused = false;
    try {
      Robot robot = make_robot(bad);
    } catch (const std::invalid_argument&) {
      refused = true;
    }
    check(refused, "a timeout of " + std::to_string(bad) + " is refused");
  }
}

void a_ticking_loop_reports_no_error_and_stops() {
  Robot robot = make_robot();
  robot.set_target_joints(offset_on_joint_1(current_q(robot), 0.3));
  for (int i = 0; i < 10; ++i) {
    check(robot.state().error == 0, "a loop that ticks reports no error");
    std::this_thread::sleep_for(TIMEOUT / 2);
  }
  check(wait_until([&] { return robot.goal().status == GoalStatus::REACHED; }), "the move is reached");
  robot.stop();
  check(link().control_returns() == 1, "the stop ended the control call");
}

void a_stalled_loop_reports_the_lost_link_and_aborts_the_move() {
  Robot robot = make_robot();
  robot.set_target_joints(offset_on_joint_1(current_q(robot), 1.5));
  std::this_thread::sleep_for(milliseconds(50));
  link().pause();
  std::this_thread::sleep_for(3 * TIMEOUT);
  const auto st = robot.state();
  check(st.error == 1, "a stalled loop reports an error");
  check(contains(st.error_message, "no robot state for 0.1 s"), "the error names the lost link: " + st.error_message);
  const auto goal = robot.goal();
  check(goal.status == GoalStatus::ABORTED, "the move in flight is aborted");
  check(contains(goal.reason, "no robot state"), "the abort names the lost link: " + goal.reason);
  link().resume();
}

void a_stop_of_a_stalled_loop_raises_without_joining() {
  Robot robot = make_robot();
  robot.set_target_joints(offset_on_joint_1(current_q(robot), 1.5));
  std::this_thread::sleep_for(milliseconds(50));
  link().pause();
  auto stopped = std::async(std::launch::async, [&] { robot.stop(); });
  const bool ended = stopped.wait_for(OUTER_LIMIT) == std::future_status::ready;
  // Free a stop that does not give up, so the suite ends and reports.
  link().resume();
  check(ended, "the stop returns while the loop stays stalled");
  bool raised = false;
  try {
    stopped.get();
  } catch (const std::runtime_error& e) {
    raised = contains(e.what(), "no robot state");
  }
  check(raised, "the stop raises and names the lost link");
}

// A stall seen through `read` ends the move; a loop that then resumes must brake, not drive on.
void check_a_resumed_loop_brakes(const std::function<void(Robot&)>& read) {
  Robot robot = make_robot();
  const Vector7d start = current_q(robot);
  robot.set_target_joints(offset_on_joint_1(start, 1.5));
  std::this_thread::sleep_for(milliseconds(100));
  link().pause();
  std::this_thread::sleep_for(3 * TIMEOUT);
  read(robot);
  const double at_stall = link().state().q[0];
  link().resume();
  check(wait_until([&] { return link().control_returns() == 1; }), "the resumed loop ends its control call");
  const double travelled = link().state().q[0] - at_stall;
  check(travelled < 0.5, "the resumed loop braked, it moved " + std::to_string(travelled) + " rad more");
  check(robot.goal().status == GoalStatus::ABORTED, "the move stays aborted");
}

void a_loop_that_resumes_after_goal_reported_the_stall_brakes() {
  check_a_resumed_loop_brakes([](Robot& robot) {
    check(robot.goal().status == GoalStatus::ABORTED, "goal() reports the stall");
  });
}

void a_loop_that_resumes_after_state_reported_the_stall_brakes() {
  check_a_resumed_loop_brakes([](Robot& robot) { check(robot.state().error == 1, "state() reports the stall"); });
}

void a_longer_timeout_does_not_flag_a_shorter_stall() {
  Robot robot = make_robot(1.0);
  robot.set_target_joints(offset_on_joint_1(current_q(robot), 1.5));
  std::this_thread::sleep_for(milliseconds(50));
  link().pause();
  std::this_thread::sleep_for(milliseconds(600));
  check(robot.state().error == 0, "a 0.6 s stall is inside a 1 s timeout");
  check(robot.goal().status == GoalStatus::IN_FLIGHT, "the move is still in flight");
  link().resume();
  robot.stop();
}

// Ruckig refuses a new plan from the end of this move to its own target.
void a_repeated_target_after_arrival_is_reached_again() {
  Robot robot = make_robot();
  const Vector7d target = (current_q(robot).array() + 0.693).matrix();
  robot.set_target_joints(target);
  check(wait_until([&] { return robot.goal().status == GoalStatus::REACHED; }), "the first move is reached");
  robot.set_target_joints(target);
  check(wait_until([&] { return robot.goal().status != GoalStatus::IN_FLIGHT; }), "the repeated move ends");
  const auto goal = robot.goal();
  check(goal.status == GoalStatus::REACHED, "the repeated move is reached, not aborted: " + goal.reason);
  robot.stop();
}

// The FR3 with bounded parts: the self-collision capsules of links 2 .. 7, a gripper body, and a pad point where
// the closed pads of a 2F-85 meet, 0.155 m along the flange z axis.
shared_kinematics::ArmModel fr3_with_gripper() {
  shared_kinematics::ArmModel model = positronic_franka::fr3_arm_model();
  for (const auto& c : model.self.capsules) model.bodies.push_back({c, shared_kinematics::Role::kRigid, c.link > 2});
  const int flange = positronic_franka::FR3_FLANGE_LINK;
  model.bodies.push_back({{"gripper", flange, {0, 0, 0.01}, {0, 0, 0.11}, 0.045}, shared_kinematics::Role::kRigid});
  model.bodies.push_back({{"pads", flange, {0, 0, 0.155}, {0, 0, 0.155}, 0.0}, shared_kinematics::Role::kPad});
  return model;
}

Robot make_bounded_robot(const std::optional<WorkspaceBounds>& bounds,
                         const positronic_franka::ControlMode& mode = InternalImpedance{}, bool clip_self = false) {
  link().reset();
  return Robot("fake", franka::RealtimeConfig::kIgnore, 1.0, mode, TIMEOUT_S, bounds, clip_self, fr3_with_gripper());
}

WorkspaceBounds a_desk(double offset = 0.02) {
  return WorkspaceBounds({{"desk", SideKind::kSurface, Eigen::Vector3d::UnitZ(), offset, 0.006, -0.005}});
}

// Unconfigured, the guard reports nothing and passes each target as it is.
void with_no_bounds_the_state_reports_no_guard_and_the_target_as_commanded() {
  Robot robot = make_robot();
  const auto before = robot.state();
  check(!robot.workspace_bounds() && !before.bounds_hash, "a new Robot has bounds");
  check(before.guard.bindings.empty() && before.guard.deepest.empty(), "a new Robot reports a guard");
  check(!before.committed_target, "a new Robot reports a committed target");
  const Vector7d target = offset_on_joint_1(current_q(robot), 0.3);
  robot.set_target_joints(target);
  check(wait_until([&] { return robot.goal().status == GoalStatus::REACHED; }), "the move is reached");
  check(arm_comes_to(robot, target), "the arm does not come to the target");
  const auto after = robot.state();
  check(after.committed_target && *after.committed_target == target, "the committed target is not the target");
  check(after.guard.bindings.empty() && !after.bounds_hash, "the guard reports a binding");
  robot.stop();
}

void bounds_given_at_the_start_are_in_force() {
  Robot robot = make_bounded_robot(a_desk());
  check(robot.state().bounds_hash == a_desk().hash(), "the state does not report the bounds of the start");
  check(robot.workspace_bounds() && robot.workspace_bounds()->hash() == a_desk().hash(), "the bounds are not kept");
}

void bounds_need_an_arm_model_with_bounded_parts() {
  bool refused = false;
  try {
    link().reset();
    Robot robot("fake", franka::RealtimeConfig::kIgnore, 1.0, InternalImpedance{}, TIMEOUT_S, a_desk());
  } catch (const std::invalid_argument& e) {
    refused = contains(e.what(), "bounded parts");
  }
  check(refused, "bounds on the FR3 with no bounded part are taken");
  Robot robot = make_robot();
  refused = false;
  try {
    robot.set_workspace_bounds(a_desk());
  } catch (const std::invalid_argument&) {
    refused = true;
  }
  check(refused && !robot.workspace_bounds(), "set_workspace_bounds takes bounds on the FR3 with no bounded part");
}

void bounds_change_at_rest_and_go_with_none() {
  Robot robot = make_bounded_robot(std::nullopt);
  robot.set_workspace_bounds(a_desk());
  check(robot.state().bounds_hash == a_desk().hash(), "the bounds are not set before the first move");
  robot.set_target_joints(offset_on_joint_1(current_q(robot), 0.3));
  check(wait_until([&] { return robot.goal().status == GoalStatus::REACHED; }), "the move is reached");
  check(wait_until([&] { return robot.state().dq.cwiseAbs().maxCoeff() == 0.0; }), "the arm comes to rest");
  robot.set_workspace_bounds(a_desk(0.03));
  check(robot.state().bounds_hash == a_desk(0.03).hash(), "the bounds are not set at rest in a running loop");
  robot.set_workspace_bounds(std::nullopt);
  check(!robot.state().bounds_hash && !robot.workspace_bounds(), "None does not remove the bounds");
  robot.stop();
}

void a_bounds_change_during_a_move_is_refused() {
  Robot robot = make_bounded_robot(std::nullopt);
  robot.set_workspace_bounds(a_desk());
  robot.set_target_joints(offset_on_joint_1(current_q(robot), 1.0));
  bool raised = false;
  try {
    robot.set_workspace_bounds(std::nullopt);
  } catch (const std::runtime_error& e) {
    raised = contains(e.what(), "only at rest");
  }
  check(raised, "a change during a move is taken");
  check(robot.state().bounds_hash == a_desk().hash(), "a refused change changed the bounds");
  robot.stop();
}

// An FR3 posture with the shoulder back and the elbow folded: link 5 is 5.1 cm from link 2. The straight joint move
// to it from the fake's start passes 0.09 m at 87 % of the way.
Vector7d shoulder_back_elbow_folded() {
  Vector7d q;
  q << 0.98877418, -1.15661502, -0.96312511, -2.93868756, -0.990777671, 2.05664039, 1.14565432;
  return q;
}

double least_clearance(const Vector7d& q) {
  double least = std::numeric_limits<double>::infinity();
  for (const auto& pair : positronic_franka::self_collision_clearances(q)) least = std::min(least, pair.clearance);
  return least;
}

double least_clearance_on_the_move(const Vector7d& from, const Vector7d& to) {
  double least = least_clearance(from);
  for (int i = 1; i <= 1000; ++i) least = std::min(least, least_clearance(from + i / 1000.0 * (to - from)));
  return least;
}

Robot make_clipping_robot(const positronic_franka::ControlMode& mode = InternalImpedance{}) {
  link().reset();
  return Robot("fake", franka::RealtimeConfig::kIgnore, 1.0, mode, TIMEOUT_S, std::nullopt,
               /*clip_joint_targets_self_collision=*/true);
}

// Unconfigured, a joint target is not checked for self-collision.
void with_no_clip_a_joint_target_into_the_arm_is_reached_as_commanded() {
  Robot robot = make_robot();
  const Vector7d target = shoulder_back_elbow_folded();
  robot.set_target_joints(target);
  check(wait_until([&] { return robot.goal().status != GoalStatus::IN_FLIGHT; }), "the move ends");
  check(robot.goal().status == GoalStatus::REACHED, "the move is not reached");
  check(arm_comes_to(robot, target), "the arm does not come to the target");
  const auto st = robot.state();
  check(*st.committed_target == target, "the committed target is not the target");
  check(st.guard.bindings.empty(), "the guard reports a binding");
  robot.stop();
}

void the_clip_stops_a_joint_target_at_the_self_collision_floor_and_the_move_ends_bounded() {
  Robot robot = make_clipping_robot();
  const Vector7d start = current_q(robot), target = shoulder_back_elbow_folded();
  robot.set_target_joints(target);
  check(wait_until([&] { return robot.goal().status != GoalStatus::IN_FLIGHT; }), "the move ends");
  const auto goal = robot.goal();
  check(goal.status == GoalStatus::BOUNDED, "the shortened move does not end BOUNDED: " + goal.reason);
  const auto st = robot.state();
  const Vector7d committed = *st.committed_target;
  const double part = (committed - start).norm() / (target - start).norm();
  check(part > 0.8 && part < 0.9, "the committed target is at " + std::to_string(part) + " of the move, not 0.87");
  check(least_clearance_on_the_move(start, committed) >= positronic_franka::IK_SELF_COLLISION_CLEARANCE -
                                                             positronic_franka::IK_SELF_COLLISION_PATH_TOLERANCE,
        "the committed move takes two links closer than the floor");
  check(arm_comes_to(robot, committed), "the arm does not come to the committed target");
  check(st.guard.bindings.size() == 1 && st.guard.bindings.at("self") == 1, "the binding is not counted at 'self'");
  robot.stop();
}

void a_clear_joint_target_under_the_clip_is_reached() {
  Robot robot = make_clipping_robot();
  const Vector7d target = offset_on_joint_1(current_q(robot), 0.3);
  robot.set_target_joints(target);
  check(wait_until([&] { return robot.goal().status != GoalStatus::IN_FLIGHT; }), "the move ends");
  check(robot.goal().status == GoalStatus::REACHED, "a clear move does not end REACHED");
  const auto st = robot.state();
  check(*st.committed_target == target && st.guard.bindings.empty(), "the clip changed a clear target");
  robot.stop();
}

// The torque loop's reference moves from where it is, so the clip starts there and the reference ends at the
// committed target.
void under_software_impedance_the_reference_stops_at_the_committed_target() {
  Robot robot = make_clipping_robot(SoftwareImpedance{});
  robot.set_target_joints(shoulder_back_elbow_folded());
  const Vector7d committed = *robot.state().committed_target;
  check(committed != shoulder_back_elbow_folded(), "the clip let the target through");
  check(wait_until([&] { return robot.state().q_d == committed; }),
        "the reference stops short of the committed target");
  robot.stop();
}

// A front wall 0.42 m ahead of the base. The pad point starts 0.32 m ahead of the base.
WorkspaceBounds a_front_wall() {
  return WorkspaceBounds({{"front", SideKind::kSurface, -Eigen::Vector3d::UnitX(), 0.42, 0.0, -0.005}});
}

// From the fake's start, joints 2 and 4 lean the arm forward and take the pad point 0.48 m ahead of the base.
Vector7d leaning_forward(const Vector7d& start) {
  Vector7d q = start;
  q(1) += 0.8;
  q(3) += 0.4;
  return q;
}

// The least, over the moves from `from` to 1000 points of the way to `to`, of a part's distance past its clip floor
// at a side: its floor, or its distance at `from` when that is less.
double least_to_floor_on_the_move(const WorkspaceBounds& bounds, const Vector7d& from, const Vector7d& to) {
  shared_kinematics::ArmModel model = fr3_with_gripper();
  model.self.pairs.clear();
  const shared_kinematics::Guard guard(model, bounds.sides());
  const auto at_from = guard.distances(from).planes;
  double least = std::numeric_limits<double>::infinity();
  for (int i = 0; i <= 1000; ++i) {
    const auto rows = guard.distances(from + i / 1000.0 * (to - from)).planes;
    for (std::size_t k = 0; k < rows.size(); ++k)
      least = std::min(least, rows[k].distance - std::min(rows[k].floor, at_from[k].distance));
  }
  return least;
}

void a_joint_target_past_a_side_stops_at_the_side_and_the_move_ends_bounded() {
  Robot robot = make_bounded_robot(a_front_wall());
  const Vector7d start = current_q(robot), target = leaning_forward(start);
  robot.set_target_joints(target);
  check(wait_until([&] { return robot.goal().status != GoalStatus::IN_FLIGHT; }), "the move ends");
  const auto goal = robot.goal();
  check(goal.status == GoalStatus::BOUNDED, "the move to a target past the wall does not end BOUNDED: " + goal.reason);
  const auto st = robot.state();
  const Vector7d committed = *st.committed_target;
  check(committed != target, "the clip let the target through");
  check(least_to_floor_on_the_move(a_front_wall(), start, committed) >=
            -positronic_franka::IK_SELF_COLLISION_PATH_TOLERANCE,
        "the committed move takes a part past its floor at the wall");
  check(least_to_floor_on_the_move(a_front_wall(), start, target) < -0.01, "the target is not past the wall");
  check(st.guard.bindings.size() == 1 && st.guard.bindings.at("front") == 1, "the binding is not counted at the wall");
  check(arm_comes_to(robot, committed), "the arm does not come to the committed target");
  robot.stop();
}

// With bounds but no self-collision clip, a joint target is checked against the box alone.
void with_bounds_and_no_clip_a_joint_target_is_not_checked_for_self_collision() {
  Robot robot = make_bounded_robot(a_desk(1.0));
  const Vector7d target = shoulder_back_elbow_folded();
  robot.set_target_joints(target);
  check(wait_until([&] { return robot.goal().status != GoalStatus::IN_FLIGHT; }), "the move ends");
  check(robot.goal().status == GoalStatus::REACHED, "the move into the arm does not end REACHED");
  check(*robot.state().committed_target == target && robot.state().guard.bindings.empty(),
        "the guard checked a self pair");
  robot.stop();
}

positronic_franka::ToolKinematics fake_fr3_flange(const Vector7d& q) {
  std::array<double, 7> joints;
  Eigen::Map<Vector7d>(joints.data()) = q;
  const auto flange = franka::fake::fr3_flange(joints);
  return {Eigen::Map<const Eigen::Matrix4d>(flange.pose.data()),
          Eigen::Map<const positronic_franka::SpatialJacobian>(flange.zero_jacobian.data())};
}

Vector7d flange_pose_at(const Vector7d& q) {
  const Eigen::Matrix4d T = fake_fr3_flange(q).pose;
  const Eigen::Quaterniond turn(Eigen::Matrix3d(T.topLeftCorner<3, 3>()));
  Vector7d pose;
  pose << T.topRightCorner<3, 1>(), turn.w(), turn.x(), turn.y(), turn.z();
  return pose;
}

// With no bounds, the IK of the Robot is the IK of the FR3 self pairs.
void with_no_bounds_the_ik_is_the_ik_of_the_fr3_self_pairs() {
  Robot robot = make_bounded_robot(std::nullopt);
  link().use_kinematics(franka::fake::Kinematics::kFr3);
  const Vector7d start = current_q(robot);
  for (const Vector7d& q_target : {leaning_forward(start), shoulder_back_elbow_folded()}) {
    const Vector7d pose = flange_pose_at(q_target);
    const Vector7d ours = robot.inverse_kinematics_with_limits(pose, start, 1.0);
    check(ours == positronic_franka::solve_pose_ik(fake_fr3_flange, pose, start, 1.0), "the IK differs");
  }
}

// The IK pushes the parts away from the wall and stops the move at its floor.
void the_ik_keeps_the_parts_inside_the_wall_and_moves_toward_the_target() {
  Robot robot = make_bounded_robot(a_front_wall());
  link().use_kinematics(franka::fake::Kinematics::kFr3);
  const Vector7d start = current_q(robot);
  const Vector7d pose = flange_pose_at(leaning_forward(start));
  const Vector7d q = robot.inverse_kinematics_with_limits(pose, start, 1.0);
  const double kept = least_to_floor_on_the_move(a_front_wall(), start, q);
  check(kept >= -positronic_franka::IK_SELF_COLLISION_PATH_TOLERANCE, "the IK move takes a part past its floor");
  check(kept < 0.02, "the IK answer stays " + std::to_string(kept) + " m from the floor, not near it");
  const double before = (flange_pose_at(start).head<3>() - pose.head<3>()).norm();
  const double after = (flange_pose_at(q).head<3>() - pose.head<3>()).norm();
  check(after < before - 0.05, "the IK answer does not move the flange toward the target");
}

// The torque loop slows the reference near the wall, keeps it inside the floors, and brings it close to the committed
// target, where the clip stopped it.
void the_torque_loop_slows_the_reference_near_a_side_and_brings_it_to_the_committed_target() {
  Robot robot = make_bounded_robot(a_front_wall(), SoftwareImpedance{});
  const Vector7d start = current_q(robot);
  robot.set_target_joints(leaning_forward(start));
  const Vector7d committed = *robot.state().committed_target;
  // Near the floor the scale lets the reference close in at 2/s of its distance, so give it 4 s of loop time.
  std::vector<Vector7d> refs;
  check(wait_until([&] {
          refs.push_back(robot.state().q_d);
          return robot.state().time >= 4.0;
        }, milliseconds(60000)),
        "the loop does not tick for 4 s");
  const auto st = robot.state();
  const double gap = (st.q_d - committed).cwiseAbs().maxCoeff();
  check(gap < 1e-3, "the reference is " + std::to_string(gap) + " rad from the committed target after 4 s");
  check(st.guard.slowed_ticks.count("front") && st.guard.slowed_ticks.at("front") > 0,
        "the speed scale did not slow the reference at the wall");
  for (const Vector7d& ref : refs)
    check(least_to_floor_on_the_move(a_front_wall(), start, ref) >=
              -positronic_franka::IK_SELF_COLLISION_PATH_TOLERANCE,
          "the reference passes a floor at the wall");
  robot.stop();
}

// At a slow joint cap near the wall, the reference that takes the committed target keeps to the cap at every tick.
void near_a_side_the_torque_loop_reference_keeps_to_its_joint_cap() {
  constexpr double kCap = 0.01;
  std::array<double, 7> slow;
  slow.fill(kCap);
  link().reset();
  const Vector7d at_start = Eigen::Map<const Vector7d>(link().state().q.data());
  // A front wall where the part nearest to its floor at the start is 4 mm from it.
  const auto wall_at = [](double x) {
    return WorkspaceBounds({{"front", SideKind::kSurface, -Eigen::Vector3d::UnitX(), x, 0.0, -0.005}});
  };
  shared_kinematics::ArmModel model = fr3_with_gripper();
  model.self.pairs.clear();
  double least = std::numeric_limits<double>::infinity();
  for (const auto& row : shared_kinematics::Guard(model, wall_at(1.0).sides()).distances(at_start).planes)
    least = std::min(least, row.distance - row.floor);
  const WorkspaceBounds wall = wall_at(1.0 - least + 0.004);
  Robot robot = make_bounded_robot(wall, SoftwareImpedance(positronic_franka::DEFAULT_MAX_SPRING_TORQUE_RATE, slow));
  const Vector7d start = current_q(robot);
  Vector7d target = start;
  target(1) += 0.04;
  target(3) += 0.02;
  robot.set_target_joints(target);
  const Vector7d committed = *robot.state().committed_target;
  check(committed != target, "the wall does not stop the move");
  check((committed - start).cwiseAbs().maxCoeff() < 0.04, "the committed target is too far for the slow cap");
  std::vector<std::pair<double, Vector7d>> refs;
  const bool arrived = wait_until([&] {
    const auto st = robot.state();
    refs.emplace_back(st.time, st.q_d);
    return st.q_d == committed;
  }, milliseconds(90000));
  check(arrived, "the reference is " + std::to_string((refs.back().second - committed).cwiseAbs().maxCoeff()) +
                     " rad from the committed target after " + std::to_string(refs.back().first) + " s");
  for (std::size_t i = 1; i < refs.size(); ++i) {
    const double ticks = std::round((refs[i].first - refs[i - 1].first) * 1000.0);
    const double step = (refs[i].second - refs[i - 1].second).cwiseAbs().maxCoeff();
    check(step <= kCap * ticks * 1e-3 + 1e-12,
          "the reference moves " + std::to_string(step) + " rad in " + std::to_string(ticks) + " ticks");
  }
  robot.stop();
}

// Unconfigured, the torque loop moves the reference at its joint cap and slows nothing.
void with_no_bounds_the_torque_loop_moves_the_reference_at_its_joint_cap() {
  Robot robot = make_bounded_robot(std::nullopt, SoftwareImpedance{});
  const Vector7d start = current_q(robot);
  robot.set_target_joints(offset_on_joint_1(start, 1.0));
  check(wait_until([&] { return robot.state().time >= 0.2; }), "the loop ticks for 200 ms");
  const auto st = robot.state();
  const double speed = (st.q_d(0) - start(0)) / st.time;
  check(std::abs(speed - positronic_franka::DEFAULT_MAX_REFERENCE_SPEED[0]) < 0.01,
        "joint 1 moves at " + std::to_string(speed) + " rad/s, not its 1.5 rad/s cap");
  check(st.guard.slowed_ticks.empty() && st.guard.deepest.empty(), "the loop reports a guard");
  robot.stop();
}

// The state keeps, per side, the deepest pass of a part past its floor at the measured joints.
void the_state_keeps_the_deepest_pass_past_a_floor() {
  const WorkspaceBounds wall({{"front", SideKind::kSurface, -Eigen::Vector3d::UnitX(), 0.30, 0.0, -0.005}});
  Robot robot = make_bounded_robot(wall);
  const Vector7d start = current_q(robot);
  shared_kinematics::ArmModel model = fr3_with_gripper();
  model.self.pairs.clear();
  double deepest = 0.0;
  for (const auto& row : shared_kinematics::Guard(model, wall.sides()).distances(start).planes)
    deepest = std::max(deepest, row.floor - row.distance);
  check(deepest > 0.01, "the start is not past the wall");
  robot.set_target_joints(start);
  check(wait_until([&] { return robot.state().guard.deepest.count("front") > 0; }), "the state keeps no pass");
  check(std::abs(robot.state().guard.deepest.at("front") - deepest) < 1e-9,
        "the deepest pass is not the pass at start");
  robot.stop();
}

// Moves joint 1 under software impedance with the tool point capped at `max_tool_point_speed` m/s, and returns
// the joint 1 reference speed, rad/s, over the first 200 ms or more.
double joint_1_reference_speed_under_tool_point_cap(double max_tool_point_speed) {
  link().reset();
  Robot robot("fake", franka::RealtimeConfig::kIgnore, 1.0,
              SoftwareImpedance(positronic_franka::DEFAULT_MAX_SPRING_TORQUE_RATE,
                                positronic_franka::DEFAULT_MAX_REFERENCE_SPEED,
                                positronic_franka::DEFAULT_STIFFNESS_POINT, max_tool_point_speed,
                                positronic_franka::DEFAULT_TOOL_POINT),
              TIMEOUT_S);
  const Vector7d start = current_q(robot);
  robot.set_target_joints(offset_on_joint_1(start, 1.0));
  check(wait_until([&] { return robot.state().time >= 0.2; }), "the loop ticks for 200 ms");
  const auto st = robot.state();
  robot.stop();
  return (st.q_d(0) - start(0)) / st.time;
}

Vector7d sample_joints() {
  Vector7d q;
  q << 0.3, -0.5, 0.2, -2.0, 0.4, 1.9, -0.6;
  return q;
}

void the_fake_fr3_kinematics_give_the_fr3_flange_and_its_zero_jacobian() {
  Robot robot = make_robot();
  link().use_kinematics(franka::fake::Kinematics::kFr3);
  const Vector7d q = sample_joints();
  const Eigen::Isometry3d flange = positronic_franka::fr3_link_frames(q)[8];
  const Vector7d pose = robot.forward_kinematics(q);
  check((pose.head<3>() - flange.translation()).norm() < 1e-12, "the fake flange position is the FR3 flange");
  const Eigen::Quaterniond turn(pose(3), pose(4), pose(5), pose(6));
  check(turn.angularDistance(Eigen::Quaterniond(flange.linear())) < 1e-9, "the fake flange turn is the FR3 flange");
  const auto J = robot.zero_jacobian(q, positronic_franka::DEFAULT_STIFFNESS_POINT);
  constexpr double h = 1e-6;
  for (int i = 0; i < 7; ++i) {
    Vector7d dq = Vector7d::Zero();
    dq(i) = h;
    const Eigen::Vector3d slope =
        (robot.forward_kinematics(q + dq).head<3>() - robot.forward_kinematics(q - dq).head<3>()) / (2 * h);
    check((J.block<3, 1>(0, i) - slope).norm() < 1e-6,
          "joint " + std::to_string(i + 1) + " moves the flange as J says");
  }
  const auto st = robot.state();
  check((st.flange_pose.head<3>() - positronic_franka::fr3_link_frames(st.q)[8].translation()).norm() < 1e-12,
        "the state flange is the FR3 flange at the measured joints");
}

void a_reset_gives_the_fake_its_constant_jacobian_again() {
  make_robot();
  link().use_kinematics(franka::fake::Kinematics::kFr3);
  Robot robot = make_robot();
  const auto J = robot.zero_jacobian(sample_joints(), positronic_franka::DEFAULT_STIFFNESS_POINT);
  positronic_franka::SpatialJacobian constant = positronic_franka::SpatialJacobian::Zero();
  constant(0, 0) = franka::Model::JOINT_1_LEVER_ARM_M;
  check(J == constant, "joint 1 moves every frame 0.5 m/rad along x, and nothing else moves");
}

void the_tool_point_cap_slows_a_reference_that_would_move_the_tool_faster() {
  // Joint 1 at its 1.5 rad/s cap moves the fake's tool at 0.75 m/s; a 0.7 m/s cap holds joint 1 at 1.4 rad/s.
  const double speed = joint_1_reference_speed_under_tool_point_cap(0.7);
  check(std::abs(speed - 0.7 / franka::Model::JOINT_1_LEVER_ARM_M) < 0.01,
        "joint 1 moves at " + std::to_string(speed) + " rad/s, not 1.4");
}

void a_tool_point_cap_above_the_joint_cap_leaves_the_joint_cap_in_charge() {
  const double speed = joint_1_reference_speed_under_tool_point_cap(1.0);
  check(std::abs(speed - positronic_franka::DEFAULT_MAX_REFERENCE_SPEED[0]) < 0.01,
        "joint 1 moves at " + std::to_string(speed) + " rad/s, not its 1.5 rad/s cap");
}

}  // namespace

int main() {
  const std::vector<std::pair<std::string, std::function<void()>>> tests = {
      {"the_default_control_link_timeout_is_10_s", the_default_control_link_timeout_is_10_s},
      {"a_bad_control_link_timeout_is_refused", a_bad_control_link_timeout_is_refused},
      {"a_ticking_loop_reports_no_error_and_stops", a_ticking_loop_reports_no_error_and_stops},
      {"a_stalled_loop_reports_the_lost_link_and_aborts_the_move",
       a_stalled_loop_reports_the_lost_link_and_aborts_the_move},
      {"a_stop_of_a_stalled_loop_raises_without_joining", a_stop_of_a_stalled_loop_raises_without_joining},
      {"a_loop_that_resumes_after_goal_reported_the_stall_brakes",
       a_loop_that_resumes_after_goal_reported_the_stall_brakes},
      {"a_loop_that_resumes_after_state_reported_the_stall_brakes",
       a_loop_that_resumes_after_state_reported_the_stall_brakes},
      {"a_longer_timeout_does_not_flag_a_shorter_stall", a_longer_timeout_does_not_flag_a_shorter_stall},
      {"a_repeated_target_after_arrival_is_reached_again", a_repeated_target_after_arrival_is_reached_again},
      {"the_fake_fr3_kinematics_give_the_fr3_flange_and_its_zero_jacobian",
       the_fake_fr3_kinematics_give_the_fr3_flange_and_its_zero_jacobian},
      {"a_reset_gives_the_fake_its_constant_jacobian_again", a_reset_gives_the_fake_its_constant_jacobian_again},
      {"with_no_bounds_the_state_reports_no_guard_and_the_target_as_commanded",
       with_no_bounds_the_state_reports_no_guard_and_the_target_as_commanded},
      {"bounds_given_at_the_start_are_in_force", bounds_given_at_the_start_are_in_force},
      {"bounds_need_an_arm_model_with_bounded_parts", bounds_need_an_arm_model_with_bounded_parts},
      {"bounds_change_at_rest_and_go_with_none", bounds_change_at_rest_and_go_with_none},
      {"a_bounds_change_during_a_move_is_refused", a_bounds_change_during_a_move_is_refused},
      {"with_no_clip_a_joint_target_into_the_arm_is_reached_as_commanded",
       with_no_clip_a_joint_target_into_the_arm_is_reached_as_commanded},
      {"the_clip_stops_a_joint_target_at_the_self_collision_floor_and_the_move_ends_bounded",
       the_clip_stops_a_joint_target_at_the_self_collision_floor_and_the_move_ends_bounded},
      {"a_clear_joint_target_under_the_clip_is_reached", a_clear_joint_target_under_the_clip_is_reached},
      {"under_software_impedance_the_reference_stops_at_the_committed_target",
       under_software_impedance_the_reference_stops_at_the_committed_target},
      {"a_joint_target_past_a_side_stops_at_the_side_and_the_move_ends_bounded",
       a_joint_target_past_a_side_stops_at_the_side_and_the_move_ends_bounded},
      {"with_bounds_and_no_clip_a_joint_target_is_not_checked_for_self_collision",
       with_bounds_and_no_clip_a_joint_target_is_not_checked_for_self_collision},
      {"with_no_bounds_the_ik_is_the_ik_of_the_fr3_self_pairs", with_no_bounds_the_ik_is_the_ik_of_the_fr3_self_pairs},
      {"the_ik_keeps_the_parts_inside_the_wall_and_moves_toward_the_target",
       the_ik_keeps_the_parts_inside_the_wall_and_moves_toward_the_target},
      {"the_torque_loop_slows_the_reference_near_a_side_and_brings_it_to_the_committed_target",
       the_torque_loop_slows_the_reference_near_a_side_and_brings_it_to_the_committed_target},
      {"near_a_side_the_torque_loop_reference_keeps_to_its_joint_cap",
       near_a_side_the_torque_loop_reference_keeps_to_its_joint_cap},
      {"with_no_bounds_the_torque_loop_moves_the_reference_at_its_joint_cap",
       with_no_bounds_the_torque_loop_moves_the_reference_at_its_joint_cap},
      {"the_state_keeps_the_deepest_pass_past_a_floor", the_state_keeps_the_deepest_pass_past_a_floor},
      {"the_tool_point_cap_slows_a_reference_that_would_move_the_tool_faster",
       the_tool_point_cap_slows_a_reference_that_would_move_the_tool_faster},
      {"a_tool_point_cap_above_the_joint_cap_leaves_the_joint_cap_in_charge",
       a_tool_point_cap_above_the_joint_cap_leaves_the_joint_cap_in_charge},
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
