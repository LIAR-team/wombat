// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/mobile_base/mobile_base.hpp"
#include "kennel/robot_sim.hpp"
#include "wombat_core/math/angles.hpp"

namespace kennel {

Lidar2D::Lidar2D(rclcpp::Node * parent_node)
: m_clock(parent_node->get_clock()), m_logger(parent_node->get_logger())
{
  RCLCPP_INFO(m_logger, "2D lidar constructed");
}

sensor_msgs::msg::LaserScan Lidar2D::compute_laser_scan(
  const LocalizationData & data)
{
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.stamp = data.robot_pose.header.stamp;
  // Assume that the lidar is mounted on the robot center
  // TODO: make it configurable
  scan_msg.header.frame_id = "base_link";

  // Assume a 360 degree lidar
  // TODO: make it configurable
  scan_msg.angle_min = - wombat_core::PI / 2.0;
  scan_msg.angle_max = wombat_core::PI / 2.0;
  // TODO: make it configurable and realistic
  scan_msg.angle_increment = 1.0;
  scan_msg.range_min = 0.0;
  scan_msg.range_max = 20.0;

  // TODO: compute range values from the map
  for (size_t i = 0; i < 360; i++) {
    scan_msg.ranges.push_back(2.0);
  }

  return scan_msg;
}

}
