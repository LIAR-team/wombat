// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/sensor_interface.hpp"
#include "kennel/mobile_base/mobile_base.hpp"

namespace kennel
{

/**
 * @brief Class used to simulate a single, whole, robot in the
 * kennel environment
 */
class RobotSim : public rclcpp::Node
{
public:
  explicit RobotSim(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~RobotSim();

private:
  void sensors_update();

  bool load_plugins();

  std::unique_ptr<MobileBase> m_mobile_base;

  std::vector<std::shared_ptr<SensorInterface>> m_sensors;
  rclcpp::TimerBase::SharedPtr m_sensors_timer;

  pluginlib::ClassLoader<SensorInterface> m_plugin_loader{
    "kennel",
    "kennel::SensorInterface"};
};

}  // namespace kennel
