// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "rclcpp/rclcpp.hpp"

#include "kennel/sensors/sensor_base.hpp"

namespace kennel
{

bool SensorBase::register_sensor(
  rclcpp::Node * parent_node,
  const std::string & sensor_name)
{
  m_clock = parent_node->get_clock();
  m_log_interface = parent_node->get_node_logging_interface();
  m_sensor_name = sensor_name;

  return this->sensor_setup(parent_node);
}

rclcpp::Logger SensorBase::get_logger()
{
  return m_log_interface->get_logger().get_child(m_sensor_name);
}

}  // namespace kennel
