// Checks the control loop watch in control_loop_watch.hpp with a fake control loop. Needs no libfranka:
//   g++ -std=c++17 -pthread -I src tests/test_control_loop_watch.cpp -o test_control_loop_watch
//   ./test_control_loop_watch

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_loop_watch.hpp"

namespace {

using positronic_franka::ControlLoopWatch;
using positronic_franka::LoopEnd;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

struct Failure {
  std::string what;
};

void check(bool ok, const std::string& what) {
  if (!ok) throw Failure{what};
}

// Short, so the suite runs fast; long against the 1 ms tick, so a busy machine does not miss it.
constexpr milliseconds TIMEOUT{100};
// A wait that the watch must end on its own well before this.
constexpr milliseconds OUTER_LIMIT{3000};

// Stands in for libfranka's control call: it ticks every millisecond until it stalls or exits.
// A stalled loop stays inside the call, as a loop that waits for a reply that never comes.
class FakeControlLoop {
 public:
  explicit FakeControlLoop(ControlLoopWatch& watch) : watch_(watch) {
    watch_.start();
    thread_ = std::thread([this] { run_(); });
  }
  ~FakeControlLoop() {
    finish();
    thread_.join();
  }
  void stall() { mode_.store(Mode::Stall); }
  void finish() { mode_.store(Mode::Exit); }

 private:
  enum class Mode { Tick, Stall, Exit };
  void run_() {
    while (mode_.load() != Mode::Exit) {
      if (mode_.load() == Mode::Tick) watch_.tick();
      std::this_thread::sleep_for(milliseconds(1));
    }
    watch_.exited();
  }
  ControlLoopWatch& watch_;
  std::atomic<Mode> mode_{Mode::Tick};
  std::thread thread_;
};

double seconds_since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

void a_loop_that_keeps_ticking_has_not_stopped_ticking() {
  ControlLoopWatch watch(TIMEOUT);
  FakeControlLoop loop(watch);
  std::this_thread::sleep_for(3 * TIMEOUT);
  check(!watch.stopped_ticking(), "a loop that ticks every millisecond is live");
}

void a_loop_that_stalls_has_stopped_ticking_after_the_timeout() {
  ControlLoopWatch watch(TIMEOUT);
  FakeControlLoop loop(watch);
  std::this_thread::sleep_for(TIMEOUT / 2);
  loop.stall();
  std::this_thread::sleep_for(2 * TIMEOUT);
  check(watch.stopped_ticking(), "a loop with no tick for twice the timeout has stopped ticking");
}

void a_wait_gives_up_on_a_loop_that_stalls() {
  ControlLoopWatch watch(TIMEOUT);
  FakeControlLoop loop(watch);
  loop.stall();
  const auto start = Clock::now();
  auto waited = std::async(std::launch::async, [&] { return watch.wait_for_exit(); });
  const bool ended = waited.wait_for(OUTER_LIMIT) == std::future_status::ready;
  // Free a wait that does not give up, so the suite ends and reports.
  loop.finish();
  check(ended, "the wait ends on its own while the loop stays stalled");
  check(waited.get() == LoopEnd::STOPPED_TICKING, "the wait reports that the loop stopped ticking");
  check(seconds_since(start) < 10 * std::chrono::duration<double>(TIMEOUT).count(),
        "the wait gives up about one timeout after the last tick");
}

void a_wait_lets_a_ticking_loop_run_longer_than_the_timeout() {
  ControlLoopWatch watch(TIMEOUT);
  FakeControlLoop loop(watch);
  const auto start = Clock::now();
  std::thread finisher([&] {
    std::this_thread::sleep_for(4 * TIMEOUT);
    loop.finish();
  });
  const LoopEnd end = watch.wait_for_exit();
  finisher.join();
  check(end == LoopEnd::EXITED, "a loop that ticks until it exits is waited for");
  check(seconds_since(start) >= 3 * std::chrono::duration<double>(TIMEOUT).count(),
        "the wait lasted until the loop exited");
}

void a_wait_with_no_loop_returns_at_once() {
  ControlLoopWatch watch(TIMEOUT);
  const auto start = Clock::now();
  check(watch.wait_for_exit() == LoopEnd::EXITED, "no loop ran, so there is nothing to wait for");
  check(seconds_since(start) < 0.01, "the wait returned at once");
  check(!watch.stopped_ticking(), "no loop ran, so no loop stopped ticking");
}

void a_loop_that_exited_has_not_stopped_ticking() {
  ControlLoopWatch watch(TIMEOUT);
  {
    FakeControlLoop loop(watch);
    loop.finish();
    check(watch.wait_for_exit() == LoopEnd::EXITED, "the loop exited");
  }
  std::this_thread::sleep_for(2 * TIMEOUT);
  check(!watch.stopped_ticking(), "an exited loop is not a loop that stopped ticking");
}

void a_new_loop_counts_as_ticking_from_its_start() {
  ControlLoopWatch watch(TIMEOUT);
  std::this_thread::sleep_for(2 * TIMEOUT);
  watch.start();
  check(!watch.stopped_ticking(), "a loop gets one timeout for its first tick");
  std::this_thread::sleep_for(2 * TIMEOUT);
  check(watch.stopped_ticking(), "a loop that never ticks has stopped ticking after the timeout");
  watch.exited();
}

}  // namespace

int main() {
  const std::vector<std::pair<std::string, std::function<void()>>> tests = {
      {"a_loop_that_keeps_ticking_has_not_stopped_ticking", a_loop_that_keeps_ticking_has_not_stopped_ticking},
      {"a_loop_that_stalls_has_stopped_ticking_after_the_timeout",
       a_loop_that_stalls_has_stopped_ticking_after_the_timeout},
      {"a_wait_gives_up_on_a_loop_that_stalls", a_wait_gives_up_on_a_loop_that_stalls},
      {"a_wait_lets_a_ticking_loop_run_longer_than_the_timeout",
       a_wait_lets_a_ticking_loop_run_longer_than_the_timeout},
      {"a_wait_with_no_loop_returns_at_once", a_wait_with_no_loop_returns_at_once},
      {"a_loop_that_exited_has_not_stopped_ticking", a_loop_that_exited_has_not_stopped_ticking},
      {"a_new_loop_counts_as_ticking_from_its_start", a_new_loop_counts_as_ticking_from_its_start},
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
