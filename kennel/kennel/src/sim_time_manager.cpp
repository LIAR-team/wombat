// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "kennel/sim_time_manager.hpp"

namespace kennel
{

SimTimeManager::SimTimeManager(
  rclcpp::Node * parent_node,
  double real_time_factor,
  std::chrono::milliseconds sim_time_update_period)
: m_real_time_factor(real_time_factor), m_sim_time_update_period(sim_time_update_period)
{
  m_sim_time_pub = parent_node->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock",
    rclcpp::ClockQoS());
}

void SimTimeManager::run()
{
  m_should_run = true;
  auto last_loop_ts = std::chrono::steady_clock::now();
  while (rclcpp::ok() && m_should_run) {
    const auto now = std::chrono::steady_clock::now();
    const auto steady_time_since_last_loop = now - last_loop_ts;
    const auto steady_time_since_last_loop_nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(steady_time_since_last_loop);
    const auto sim_time_since_last_loop_nanoseconds = std::chrono::nanoseconds(
      static_cast<int64_t>(
        m_real_time_factor * static_cast<double>(steady_time_since_last_loop_nanoseconds.count())));
    last_loop_ts = now;

    m_sim_time += rclcpp::Duration(sim_time_since_last_loop_nanoseconds);

    // Publish the simulated time
    auto clock_msg = std::make_unique<rosgraph_msgs::msg::Clock>();
    clock_msg->clock = m_sim_time;
    m_sim_time_pub->publish(std::move(clock_msg));

    // Sleep until next iteration
    std::this_thread::sleep_for(m_sim_time_update_period);
  }
}

bool SimTimeManager::is_running() const
{
  return m_should_run;
}

void SimTimeManager::stop()
{
  m_should_run = false;
}

}  // namespace kennel
