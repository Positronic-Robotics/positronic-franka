#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace positronic_franka {

// libfranka calls the control callback every 1 ms while the link to the robot works. With no call for
// this long, the robot clock in the cached state has stopped and the loop is stuck inside libfranka.
constexpr std::chrono::milliseconds CONTROL_LINK_TIMEOUT{500};

// When one control loop last ticked, and whether it exited. A loop stuck inside libfranka neither
// ticks nor exits, and nothing can end it from outside.
class ControlLoopWatch {
 public:
  using Clock = std::chrono::steady_clock;

  explicit ControlLoopWatch(Clock::duration timeout = CONTROL_LINK_TIMEOUT) : timeout_(timeout) {}

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

  // The last thing the loop thread does.
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

  // Waits while the loop ticks. Returns true when it exits, false when it stops ticking first.
  bool wait_for_exit() {
    std::unique_lock<std::mutex> lk(mutex_);
    while (!exited_) {
      const Clock::time_point give_up = last_tick_ + timeout_;
      if (Clock::now() >= give_up) return false;
      exit_cv_.wait_until(lk, give_up);
    }
    return true;
  }

 private:
  const Clock::duration timeout_;
  mutable std::mutex mutex_;
  std::condition_variable exit_cv_;
  Clock::time_point last_tick_{};
  bool exited_ = true;
};

}  // namespace positronic_franka
