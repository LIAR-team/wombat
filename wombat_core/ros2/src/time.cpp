// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <chrono>

#include "wombat_core/ros2/time.hpp"

using std::chrono::nanoseconds;

namespace wombat_core
{

RateController::RateController(
  double frequency_hz,
  rclcpp::Clock::SharedPtr clock,
  bool start_ready)
: RateController(
  std::chrono::duration_cast<nanoseconds>(std::chrono::duration<double>(1.0 / frequency_hz)),
  clock,
  start_ready)
{}

RateController::RateController(
  nanoseconds period,
  rclcpp::Clock::SharedPtr clock,
  bool start_ready)
  : m_period(period), m_clock(clock)
{
  if (start_ready) {
    // Subtract the period to the current time to make the controller
    // immediately appear as ready
    m_last_ready_time = m_clock->now() - m_period;
  } else {
    m_last_ready_time = m_clock->now();
  }
}

bool RateController::is_ready()
{
  const auto now = m_clock->now();
  if (now - m_last_ready_time < m_period) {
    return false;
  }
  m_last_ready_time = now;
  return true;
}

bool RateController::sleep_until_ready()
{
  if (this->is_ready()) {
    return true;
  }

  // Time coming into sleep
  auto now = m_clock->now();
  // Time of next run
  auto next_ready_time = m_last_ready_time + m_period;
  // Detect backwards time flow
  if (now < m_last_ready_time) {
    // Best thing to do is to set the next_ready_time to now + period
    next_ready_time = now + m_period;
  }
  // Calculate the time to sleep
  auto time_to_sleep = next_ready_time - now;
  // If the time_to_sleep is negative or zero, don't sleep
  if (time_to_sleep <= std::chrono::seconds(0)) {
    // If an entire cycle was missed then reset next interval.
    // This might happen if the loop took more than a cycle.
    // Or if time jumps forward.
    if (now > next_ready_time + m_period) {
      m_last_ready_time = now + m_period;
    }
    // Either way do not sleep and return false
    return false;
  }
  m_clock->sleep_for(time_to_sleep);
  return this->is_ready();
}

}  // namespace wombat_core
