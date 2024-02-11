// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "wombat_core/ros2/parameters.hpp"

namespace wombat_core
{

rclcpp::ParameterValue declare_parameter_if_not_declared(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor)
{
  if (!node_parameters->has_parameter(param_name)) {
    return node_parameters->declare_parameter(param_name, default_value, parameter_descriptor);
  }
  return node_parameters->get_parameter(param_name).get_parameter_value();
}

}  // namespace wombat_core
