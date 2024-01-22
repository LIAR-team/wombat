// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/types.hpp"

namespace kennel
{

class Lidar2D
{
public:
  explicit Lidar2D(
    rclcpp::Node * parent_node);

  sensor_msgs::msg::LaserScan compute_laser_scan(
    const LocalizationData & data);

private:
  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Logger m_logger;
};

}  // namespace kennel
