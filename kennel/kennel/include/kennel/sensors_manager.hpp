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

namespace kennel
{

/**
 * @brief Class responsible for managing the various sensors equipped to a robot.
 */
class SensorsManager
{
public:
  explicit SensorsManager(rclcpp::Node * parent_node);

  void sensors_update(const localization_data_t & gt_data);

  std::chrono::milliseconds get_update_period() const;

private:
  bool load_plugins(rclcpp::Node * parent_node);

  std::vector<std::shared_ptr<SensorInterface>> m_sensors;

  pluginlib::ClassLoader<SensorInterface> m_plugin_loader {
    "kennel",
    "kennel::SensorInterface"};

  rclcpp::Logger m_logger;
  std::chrono::milliseconds m_sensors_update_period_ms;
};

}  // namespace kennel
