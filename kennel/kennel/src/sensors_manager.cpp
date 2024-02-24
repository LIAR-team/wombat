// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/sensors_manager.hpp"
#include "wombat_core/ros2/parameters.hpp"

// Macros to generate nested parameter names
#include "wombat_core/cpp/macros/concat.hpp"

#define BASE_NAME "sensors_manager"
#define BASE_PREFIX  BASE_NAME "."
#define BASE_PARAM(PARAM) WOMBAT_CORE_CONCAT_STR_LITERALS(BASE_PREFIX, PARAM)

namespace kennel
{

SensorsManager::SensorsManager(rclcpp::Node * parent_node)
: m_logger(parent_node->get_logger())
{
  const bool plugins_success = this->load_plugins(parent_node);
  if (!plugins_success) {
    throw std::runtime_error("Failed to register plugins");
  }

  m_sensors_update_period_ms = std::chrono::milliseconds(
    wombat_core::declare_parameter_if_not_declared(
      parent_node->get_node_parameters_interface(),
      BASE_PARAM("update_period_ms"),
      rclcpp::ParameterValue{10}).get<int>());

  RCLCPP_INFO(
    m_logger,
    "Sensors Manager constructed with update period %d ms",
    static_cast<int>(m_sensors_update_period_ms.count()));
}

std::chrono::milliseconds SensorsManager::get_update_period() const
{
  return m_sensors_update_period_ms;
}

bool SensorsManager::load_plugins(rclcpp::Node * parent_node)
{
  // Get name of plugins to load
  auto sensor_plugins = wombat_core::declare_parameter_if_not_declared(
    parent_node->get_node_parameters_interface(),
    BASE_PARAM("sensors"),
    rclcpp::ParameterValue(std::vector<std::string>())).get<std::vector<std::string>>();

  std::vector<std::shared_ptr<SensorInterface>> new_sensors;
  for (const auto & plugin_name : sensor_plugins) {
    // Get type of this plugin
    auto plugin_type = wombat_core::declare_parameter_if_not_declared(
      parent_node->get_node_parameters_interface(),
      BASE_PREFIX + plugin_name + ".plugin_type",
      rclcpp::ParameterValue("")).get<std::string>();
    if (plugin_type.empty()) {
      RCLCPP_ERROR(m_logger, "Plugin %s has no plugin type defined", plugin_name.c_str());
      return false;
    }

    // Load and initialize the plugin
    auto loaded_plugin = m_plugin_loader.createSharedInstance(plugin_type);
    const bool plugin_init_success = loaded_plugin->initialize_sensor(
      parent_node,
      plugin_name,
      BASE_NAME);
    if (!plugin_init_success) {
      RCLCPP_WARN(m_logger, "Failed to initialize plugin %s", plugin_name.c_str());
      return false;
    }

    // Store the initialized plugin
    new_sensors.push_back(std::move(loaded_plugin));
  }

  m_sensors = new_sensors;
  return true;
}

void SensorsManager::sensors_update(const localization_data_t & gt_data)
{
  for (auto & sensor : m_sensors) {
    sensor->produce_sensor_data(gt_data);
  }
}

}  // namespace kennel
