// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/plugin_base.hpp"
#include "wombat_core/ros2/parameters.hpp"

namespace kennel
{

bool PluginBase::initialize_plugin(
  const std::string & plugin_name,
  rclcpp::Node * parent_node,
  const std::string & params_prefix)
{
  m_clock = parent_node->get_clock();
  m_log_interface = parent_node->get_node_logging_interface();
  m_plugin_name = plugin_name;
  if (!params_prefix.empty()) {
    m_params_prefix = params_prefix + ".";
  }

  const auto default_parameters_info = this->setup_parameters();
  const bool params_success = this->declare_parameters(
    default_parameters_info,
    parent_node);
  if (!params_success) {
    RCLCPP_WARN(this->get_logger(), "Failed to setup parameters");
    return false;
  }

  return true;
}

rclcpp::Logger PluginBase::get_logger()
{
  return m_log_interface->get_logger().get_child(m_plugin_name);
}

rclcpp::ParameterValue PluginBase::get_parameter(const std::string & param_name)
{
  auto param_it = m_parameters.find(param_name);
  if (param_it == m_parameters.end()) {
    std::string exception_msg = "Plugin " + m_plugin_name + ": parameter '" + param_name + "' not found";
    throw std::out_of_range(exception_msg);
  }
  return param_it->second;
}

std::vector<default_parameter_info_t>
PluginBase::setup_parameters()
{
  return std::vector<default_parameter_info_t>();
}

bool PluginBase::post_init()
{
  return true;
}

bool PluginBase::declare_parameters(
  const std::vector<default_parameter_info_t> & default_parameters_info,
  rclcpp::Node * parent_node)
{
  for (const auto & default_info : default_parameters_info) {
    if (default_info.name.empty()) {
      RCLCPP_ERROR(get_logger(), "Setup failed: can't declare parameter without a name");
      return false;
    }

    if (m_parameters.count(default_info.name) > 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Setup failed: parameter %s already declared for this plugin",
        default_info.name.c_str());
      return false;
    }

    // Scope each parameter with the name of the plugin
    const std::string full_parameter_name =
      m_params_prefix + m_plugin_name + "." + default_info.name;

    // Parameters are declared using their fully scoped name to avoid collisions
    auto param_value = wombat_core::declare_parameter_if_not_declared(
      parent_node->get_node_parameters_interface(),
      full_parameter_name,
      default_info.value,
      default_info.descriptor);

    // Store parameter values (which can be overridden by the application) in a map
    // to make them accessible by the derived plugin class
    // Parameters are stored with their relative name to make retrieval easier
    m_parameters.emplace(default_info.name, param_value);
  }

  return true;
}

}  // namespace kennel
