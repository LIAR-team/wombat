// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/common/sensors/lidar.hpp"
#include "kennel/mobile_base/mobile_base.hpp"
#include "kennel/robot_sim.hpp"

namespace kennel
{

RobotSim::RobotSim(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_sim", options)
{
  m_mobile_base = std::make_unique<MobileBase>(this);

  const bool plugins_success = this->load_plugins();
  if (!plugins_success) {
    throw std::runtime_error("Failed to register plugins");
  }

  m_sensors_timer = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      this->sensors_update();
    });

  RCLCPP_INFO(this->get_logger(), "Robot simulation constructed");
}

bool RobotSim::load_plugins()
{
  // Get name of plugins to load
  auto sensor_plugins = this->declare_parameter(
    "sensors",
    std::vector<std::string>());

  for (const auto & plugin_name : sensor_plugins) {
    // Get type of this plugin
    auto plugin_type = this->declare_parameter(
      plugin_name + ".plugin_type",
      "");
    if (plugin_type.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Plugin %s has no plugin type defined", plugin_name.c_str());
      return false;
    }

    // Load and initialize the plugin
    auto loaded_plugin = m_plugin_loader.createSharedInstance(plugin_type);
    const bool plugin_init_success = loaded_plugin->initialize_sensor(this, plugin_name); 
    if (!plugin_init_success) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize plugin %s", plugin_name.c_str());
      return false;
    }

    // Store the initialized plugin
    m_sensors.push_back(std::move(loaded_plugin));
  }

  return true;
}

void RobotSim::sensors_update()
{
  auto gt_data = m_mobile_base->get_ground_truth_data();
  for (auto & sensor : m_sensors) {
    sensor->produce_sensor_data(gt_data);
  }
}

}  // namespace kennel
