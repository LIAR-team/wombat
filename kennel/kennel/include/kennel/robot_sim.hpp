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
#include "kennel/sensors_manager.hpp"

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

private:
  void sensors_update();

  bool load_plugins();

  // Making this a unique pointer causes a clang compilation warning:
  // [clang-analyzer-optin.cplusplus.VirtualCall]
  // https://github.com/ros/pluginlib/blob/30042370880c5a476d0a2817187aab11c1c886f6/
  // pluginlib/include/pluginlib/class_loader_imp.hpp#L112
  // I think clang is right, but the code is just logging a name so not really impactful.
  // Moreover, no idea why it happens only with unique_ptr
  std::shared_ptr<MobileBase> m_mobile_base;
  std::shared_ptr<SensorsManager> m_sensors_manager;

  rclcpp::TimerBase::SharedPtr m_sensors_timer;
  rclcpp::TimerBase::SharedPtr m_mobile_base_timer;
};

}  // namespace kennel
