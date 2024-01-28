// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/common/sensors/lidar.hpp"
#include "kennel/mobile_base/mobile_base.hpp"
#include "kennel/robot_sim.hpp"

#include "kennel/plugins/lidar_2d.hpp"

namespace kennel
{

RobotSim::RobotSim(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_sim", options)
{
  m_mobile_base = std::make_unique<MobileBase>(this);
  RCLCPP_INFO(this->get_logger(), "Robot simulation constructed");

  const std::string plugin_name = "base_scan";
  auto lidar_2d = std::make_unique<Lidar2D>();
  bool success = lidar_2d->initialize_sensor(this, plugin_name);
  if (!success) {
    throw std::runtime_error("Failed to register plugin: " + plugin_name);
  }
  m_sensors.push_back(std::move(lidar_2d));

  m_sensors_timer = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      this->sensors_update();
    });
}

void RobotSim::sensors_update()
{
  auto gt_data = m_mobile_base->get_ground_truth_data();
  for (auto & sensor : m_sensors) {
    sensor->produce_sensor_data(gt_data);
  }
}

}  // namespace kennel
