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
using positronic_franka::Vector7d;
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
