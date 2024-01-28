// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/sensors/sensor_ros2_base.hpp"
#include "kennel/types.hpp"

namespace kennel
{

class Lidar2D : public SensorRos2Base<sensor_msgs::msg::LaserScan>
{
public:
  Lidar2D() = default;

private:
  std::unique_ptr<sensor_msgs::msg::LaserScan>
  make_sensor_ros2_msg(const LocalizationData & gt_data) override;
};

}  // namespace kennel
