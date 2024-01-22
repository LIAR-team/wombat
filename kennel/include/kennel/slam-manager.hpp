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

class SlamManager
{
public:
  struct SlamData
  {
    geometry_msgs::msg::TransformStamped map_T_base;
    nav_msgs::msg::OccupancyGrid map;
  };

  SlamManager(
    rclcpp::Node * parent_node,
    const rclcpp::Duration & update_period,
    const std::string & slam_frame_id);

  std::optional<SlamData> slam_update(
    const geometry_msgs::msg::TransformStamped & gt_T_base,
    const nav_msgs::msg::OccupancyGrid & gt_map);

private:
  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Logger m_logger;

  rclcpp::Time m_last_update_time;

  rclcpp::Duration m_update_period {std::chrono::nanoseconds(0)};
  std::string m_slam_frame_id {};
};
