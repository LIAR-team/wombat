// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/mobile_base/mobile_base.hpp"
#include "kennel/robot_sim.hpp"

namespace kennel
{

RobotSim::RobotSim(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_sim", options)
{
  m_mobile_base = std::make_unique<MobileBase>(this);
  RCLCPP_INFO(this->get_logger(), "Robot simulation constructed");

  m_lidar2d = std::make_unique<Lidar2D>(this);
  m_lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "base_scan",
    rclcpp::QoS(rclcpp::KeepLast(10)));

  m_sensors_timer = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      this->sensors_update();
    });
}

void RobotSim::sensors_update()
{
  auto gt_data = m_mobile_base->get_ground_truth_data();
  auto laser_scan_msg = m_lidar2d->compute_laser_scan(gt_data);
  m_lidar_pub->publish(laser_scan_msg);
}

}  // namespace kennel
