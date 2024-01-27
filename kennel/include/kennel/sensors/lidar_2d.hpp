// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/sensors/sensor_base.hpp"
#include "kennel/types.hpp"

namespace kennel
{

class Lidar2D : public SensorBase
{
public:
  Lidar2D() = default;

  void produce_sensor_data(const LocalizationData & gt_data) override;

private:
  bool sensor_setup(rclcpp::Node * parent_node) override;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_lidar_pub;
};

}  // namespace kennel
