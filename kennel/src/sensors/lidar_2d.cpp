// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/sensors/lidar_2d.hpp"
#include "wombat_core/math/angles.hpp"

namespace kennel
{

bool Lidar2D::sensor_setup(rclcpp::Node * parent_node)
{
  m_lidar_pub = parent_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "base_scan",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  RCLCPP_INFO(this->get_logger(), "Sensor constructed");
  return true;
}

void Lidar2D::produce_sensor_data(const LocalizationData & gt_data)
{
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header.stamp = gt_data.robot_pose.header.stamp;
  // Assume that the lidar is mounted on the robot center
  // TODO: make it configurable
  scan_msg->header.frame_id = "base_link";

  // TODO: make it configurable
  scan_msg->angle_min = -wombat_core::PI / 2.0;
  scan_msg->angle_max = wombat_core::PI / 2.0;
  scan_msg->angle_increment = (scan_msg->angle_max + scan_msg->angle_min) / 360;
  scan_msg->range_min = 0.0;
  scan_msg->range_max = 20.0;

  // TODO: compute range values from the map
  for (size_t i = 0; i < 360; i++) {
    scan_msg->ranges.push_back(2.0);
  }

  m_lidar_pub->publish(std::move(scan_msg));
}

}  // namespace kennel
