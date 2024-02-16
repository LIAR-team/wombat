// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/common/types.hpp"

namespace kennel
{

/**
 * @brief Class responsible for keeping track of the pose and
 * map estimates of a mobile robot using pseudo-SLAM algorithms
 */
class SlamManager
{
public:
  SlamManager(
    rclcpp::Node * parent_node,
    const rclcpp::Duration & update_period,
    std::string slam_frame_id);

  std::optional<localization_data_t> slam_update(
    const localization_data_t & gt_data);

private:
  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Logger m_logger;

  rclcpp::Time m_last_update_time;

  rclcpp::Duration m_update_period {std::chrono::nanoseconds(0)};
  std::string m_slam_frame_id {};
};

}  // namespace kennel
