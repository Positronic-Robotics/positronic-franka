#pragma once

#include <algorithm>
#include <array>
#include <iostream>

#include <ruckig/ruckig.hpp>

#include "impedance_law.hpp"

namespace positronic_franka {

// Panda joint limits (rad/s, rad/s^2, rad/s^3)
constexpr std::array<double, 7> PANDA_BASE_VELOCITY_LIMITS = {2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
constexpr std::array<double, 7> PANDA_BASE_ACCELERATION_LIMITS = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
constexpr std::array<double, 7> PANDA_BASE_JERK_LIMITS = {5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0};

// Ruckig trajectory generator, evaluated at libfranka's measured period: fixed 1 ms steps produce
// velocity and acceleration discontinuities under non-RT scheduling.
class TrajectoryGenerator {
 public:
  static constexpr double NOMINAL_DT = 1.0 / 1000.0;

  explicit TrajectoryGenerator(double dynamics_factor) {
    input_.synchronization = ruckig::Synchronization::Time;
    input_.target_velocity.fill(0.0);
    input_.target_acceleration.fill(0.0);
    for (size_t i = 0; i < 7; ++i) {
      input_.max_velocity[i] = PANDA_BASE_VELOCITY_LIMITS[i] * dynamics_factor;
      input_.max_acceleration[i] = PANDA_BASE_ACCELERATION_LIMITS[i] * dynamics_factor;
      input_.max_jerk[i] = PANDA_BASE_JERK_LIMITS[i] * dynamics_factor;
    }
  }

  // libfranka's joint motion generator starts at rest at q_d. A plan seeded with the measured velocity
  // commands a velocity step on the first tick, and the robot stops with an acceleration discontinuity.
  void start_at_rest(const std::array<double, 7>& q_d) {
    input_.control_interface = ruckig::ControlInterface::Position;
    input_.current_position = q_d;
    input_.current_velocity.fill(0.0);
    input_.current_acceleration.fill(0.0);
    input_.target_position = q_d;
    replan_();
  }

  // Returns whether the plan to the new target was accepted; on failure the previous plan keeps playing.
  bool set_target(const Vector7d& target) {
    input_.control_interface = ruckig::ControlInterface::Position;
    for (size_t i = 0; i < 7; ++i) input_.target_position[i] = target[i];
    return replan_();
  }

  // Brakes every joint to zero speed, and the arm rests where the brake ends. A plan back to the position
  // at the stop call is refused often, and a refused plan keeps the arm going to the old target.
  void brake_to_rest() {
    input_.control_interface = ruckig::ControlInterface::Velocity;
    replan_();
  }

  // Advances the trajectory by `dt_s` seconds and returns the position. libfranka reports a zero period
  // on the first tick, and that tick must command the start position.
  std::array<double, 7> step(double dt_s) {
    if (!planned_) return input_.current_position;

    cumulative_time_ += std::max(dt_s, 0.0);

    double t = std::min(cumulative_time_, duration_);
    std::array<double, 7> pos, vel, acc;
    trajectory_.at_time(t, pos, vel, acc);

    input_.current_position = pos;
    input_.current_velocity = vel;
    input_.current_acceleration = acc;

    if (cumulative_time_ >= duration_) active_ = false;
    return pos;
  }

  bool active() const { return active_; }

  // How long the planned trajectory runs, in seconds. The move deadline adds it to the settle
  // budget, so a slow `relative_dynamics_factor` (down to 0.0001) is not cut off by a flat constant.
  double duration() const { return duration_; }

 private:
  bool replan_() {
    // Swap only on success: evaluating a default-constructed trajectory feeds NaN into libfranka and
    // kills the control thread. On failure the previous plan keeps playing.
    ruckig::Trajectory<7> next;
    auto result = otg_.calculate(input_, next);
    if (result < 0) {
      std::cerr << "Ruckig trajectory planning failed (error " << static_cast<int>(result) << ")" << std::endl;
      return false;
    }
    trajectory_ = next;
    planned_ = true;
    cumulative_time_ = 0.0;
    duration_ = trajectory_.get_duration();
    active_ = true;
    return true;
  }

  ruckig::Ruckig<7> otg_{NOMINAL_DT};
  ruckig::InputParameter<7> input_;
  ruckig::Trajectory<7> trajectory_;
  double cumulative_time_ = 0.0;
  double duration_ = 0.0;
  bool active_ = false;
  bool planned_ = false;
};

}  // namespace positronic_franka
