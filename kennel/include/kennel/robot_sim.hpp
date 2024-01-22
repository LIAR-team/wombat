// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "kennel/mobile_base/mobile_base.hpp"
#include "kennel/sensors/lidar_2d.hpp"

namespace kennel {

class RobotSim : public rclcpp::Node
{
public:
  explicit RobotSim(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void sensors_update();

  std::unique_ptr<MobileBase> m_mobile_base;

  std::unique_ptr<Lidar2D> m_lidar2d;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_lidar_pub;
  rclcpp::TimerBase::SharedPtr m_sensors_timer;
};

}
