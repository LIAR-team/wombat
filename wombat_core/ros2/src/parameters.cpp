// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <iterator>

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

std::optional<rclcpp::Parameter> get_parameter_for_node(
  const std::string & param_name,
  const rclcpp::ParameterMap & parameter_map,
  const std::string & fully_qualified_node_name)
{
  auto params_for_node = rclcpp::parameters_from_map(
    parameter_map,
    fully_qualified_node_name.c_str());

  auto param_rit = std::find_if(
    std::reverse_iterator(params_for_node.end()),
    std::reverse_iterator(params_for_node.begin()),
    [&param_name](const rclcpp::Parameter & p) {
      return param_name == p.get_name();
    });

  if (param_rit == std::reverse_iterator(params_for_node.begin())) {
    return std::nullopt;
  }
  return *param_rit;
}

bool update_parameter_map(
  rclcpp::ParameterMap & parameter_map,
  const std::string & fully_qualified_node_name,
  const std::string & param_name,
  const rclcpp::ParameterValue & param_value,
  bool allow_override)
{
  // Check if parameter is already present
  if (!allow_override) {
    auto maybe_param = get_parameter_for_node(
      param_name,
      parameter_map,
      fully_qualified_node_name);
    if (maybe_param) {
      return false;
    }
  }

  // Get (or create) parameter vector for fqn
  auto node_params_it = parameter_map.find(fully_qualified_node_name);
  if (node_params_it == parameter_map.end()) {
    bool success = false;
    std::tie(node_params_it, success) =
      parameter_map.insert(std::make_pair(fully_qualified_node_name, std::vector<rclcpp::Parameter>()));
    if (!success) {
      return false;
    }
  }

  // Add the parameter
  auto & node_params = node_params_it->second;
  node_params.push_back(rclcpp::Parameter(param_name, param_value));
  return true;
}

bool set_parameters_from_map(
  const rclcpp::ParameterMap & parameter_map,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ifc,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_ifc)
{
  const auto node_parameters = rclcpp::parameters_from_map(
    parameter_map,
    node_base_ifc->get_fully_qualified_name());

  const auto params_result = node_parameters_ifc->set_parameters(node_parameters);

  // Check that all parameters have been successfully set
  for (const auto & result : params_result) {
    if (!result.successful) {
      return false;
    }
  }

  return true;
}

}  // namespace wombat_core
