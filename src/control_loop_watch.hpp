#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace positronic_franka {

// How a wait for a control loop ended.
enum class LoopEnd { EXITED, STOPPED_TICKING };

// When one control loop last ticked, and whether it exited. libfranka calls the control callback every
// 1 ms while the link to the robot works, so a loop with no tick for `timeout` is stuck inside libfranka.
// Such a loop neither ticks nor exits, and nothing can end it from outside.
class ControlLoopWatch {
 public:
  using Clock = std::chrono::steady_clock;

  explicit ControlLoopWatch(Clock::duration timeout) : timeout_(timeout) {}

  // A new loop starts. It gets one timeout for its first tick.
  void start() {
    std::lock_guard<std::mutex> lk(mutex_);
    last_tick_ = Clock::now();
    exited_ = false;
  }

  void tick() {
    std::lock_guard<std::mutex> lk(mutex_);
    last_tick_ = Clock::now();
  }

  // Call it last in the loop thread: a waiter joins the thread as soon as this returns.
  void exited() {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      exited_ = true;
    }
    exit_cv_.notify_all();
  }

  // A loop runs and has not ticked for the timeout.
  bool stopped_ticking() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return !exited_ && Clock::now() - last_tick_ >= timeout_;
  }

  // Waits while the loop ticks, until it exits or stops ticking.
  LoopEnd wait_for_exit() {
    std::unique_lock<std::mutex> lk(mutex_);
    while (!exited_) {
      const Clock::time_point give_up = last_tick_ + timeout_;
      if (Clock::now() >= give_up) return LoopEnd::STOPPED_TICKING;
      exit_cv_.wait_until(lk, give_up);
    }
    return LoopEnd::EXITED;
  }

 private:
  const Clock::duration timeout_;
  mutable std::mutex mutex_;
  std::condition_variable exit_cv_;
  Clock::time_point last_tick_{};
  bool exited_ = true;
};

}  // namespace positronic_franka
