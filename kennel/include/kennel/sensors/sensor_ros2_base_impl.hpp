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

#include "kennel/sensors/sensor_ros2_base.hpp"

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
    return false;
  }

  m_sensor_publisher = parent_node->create_publisher<MsgT>(
    m_sensor_name,
    rclcpp::QoS(rclcpp::KeepLast(10)));

  RCLCPP_INFO(this->get_logger(), "Sensor constructed");
  return true;
}

template<typename MsgT>
void SensorRos2Base<MsgT>::produce_sensor_data(const LocalizationData & gt_data)
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
std::vector<DefaultParameterInfo>
SensorRos2Base<MsgT>::setup_parameters()
{
  return std::vector<DefaultParameterInfo>();
}

template<typename MsgT>
bool SensorRos2Base<MsgT>::declare_parameters(
  const std::vector<DefaultParameterInfo> & default_parameters_info,
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

    auto param_value = parent_node->declare_parameter(
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
