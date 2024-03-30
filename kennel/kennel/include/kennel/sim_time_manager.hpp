// Copyright 2024 Soragna Alberto.

#pragma once

#include <atomic>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace kennel
{

/**
 * @brief Class to publish simulated time.
 */
class SimTimeManager
{
public:
  /**
   * @brief This object store and publishes a simulated clock
   * @param parent_node ROS 2 node used to construct the sim time publisher
   * @param real_time_factor the multiplicative factor denoting how faster
   * (or slower) than real time should the simulated clock update.
   * @param sim_time_update_period the update period of the simulated clock.
   * This denotes the update and publish frequency.
   * @note The simulated clock resolution will be equal to the product of these
   * two values.
   */
  SimTimeManager(
    rclcpp::Node * parent_node,
    double real_time_factor,
    std::chrono::milliseconds sim_time_update_period);

  /**
   * @brief Blocking function to publish simulated time.
   *  Will run as long as rclcpp context is valid or told to stop
   */
  void run();

  /**
   * @brief Check whether the time manager is currently running
   * @return true if it's running
   */
  bool is_running() const;

  /**
   * @brief Indicate to the @sa run() function to return.
   */
  void stop();

private:
  rclcpp::Time m_sim_time {0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_sim_time_pub;
  double m_real_time_factor {0.0};
  std::chrono::milliseconds m_sim_time_update_period {};

  std::atomic<bool> m_should_run {false};
};

}  // namespace kennel
