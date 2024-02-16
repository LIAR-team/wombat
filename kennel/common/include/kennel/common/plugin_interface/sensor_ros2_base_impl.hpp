// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/sensor_ros2_base.hpp"
#include "wombat_core/ros2/parameters.hpp"

namespace kennel
{

template<typename MsgT>
bool SensorRos2Base<MsgT>::initialize_sensor(
  rclcpp::Node * parent_node,
  const std::string & sensor_name)
{
  m_clock = parent_node->get_clock();
  m_log_interface = parent_node->get_node_logging_interface();
  m_sensor_name = sensor_name;

  const auto default_parameters_info = this->setup_parameters();
  const bool params_success = this->declare_parameters(
    default_parameters_info,
    parent_node);
  if (!params_success) {
    RCLCPP_WARN(this->get_logger(), "Failed to setup parameters");
    return false;
  }

  // TODO: we should allow differentiating the sensor name from the topic name
  m_sensor_publisher = parent_node->create_publisher<MsgT>(
    m_sensor_name,
    rclcpp::SensorDataQoS());

  const bool post_init_success = this->post_init();
  if (!post_init_success) {
    RCLCPP_WARN(this->get_logger(), "Failed to run post-init routine");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Sensor initialized");
  return true;
}

template<typename MsgT>
void SensorRos2Base<MsgT>::produce_sensor_data(const localization_data_t & gt_data)
{
  auto sensor_msg = make_sensor_ros2_msg(gt_data);
  m_sensor_publisher->publish(std::move(sensor_msg));
}

template<typename MsgT>
rclcpp::Logger SensorRos2Base<MsgT>::get_logger()
{
  return m_log_interface->get_logger().get_child(m_sensor_name);
}

template<typename MsgT>
rclcpp::ParameterValue SensorRos2Base<MsgT>::get_parameter(const std::string & param_name)
{
  auto param_it = m_parameters.find(param_name);
  if (param_it == m_parameters.end()) {
    std::string exception_msg = "Parameter " + param_name + " not found";
    throw std::out_of_range(exception_msg);
  }
  return param_it->second;
}

template<typename MsgT>
std::vector<default_parameter_info_t>
SensorRos2Base<MsgT>::setup_parameters()
{
  return std::vector<default_parameter_info_t>();
}

template<typename MsgT>
bool SensorRos2Base<MsgT>::post_init()
{
  return true;
}

template<typename MsgT>
bool SensorRos2Base<MsgT>::declare_parameters(
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

    // Scope each parameter with the name of the sensor
    const std::string full_name = m_sensor_name + "." + default_info.name;

    auto param_value = wombat_core::declare_parameter_if_not_declared(
      parent_node->get_node_parameters_interface(),
      default_info.name,
      default_info.value,
      default_info.descriptor);

    // Store parameter values (which can be overridden by the application) in a map
    // to make them accessible by the derived plugin class
    m_parameters.emplace(default_info.name, param_value);
  }

  return true;
}

}  // namespace kennel
