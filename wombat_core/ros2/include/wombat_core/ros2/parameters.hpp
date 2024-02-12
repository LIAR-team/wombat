// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>
#include <unordered_set>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wombat_core
{

namespace detail
{
struct params_comparator_t
{
  bool operator()(const rclcpp::Parameter & lhs, const rclcpp::Parameter & rhs) const
  {
    return lhs.get_name() < rhs.get_name();
  }
};
}  // namespace detail

/// @brief Alias for a set of parameters indexed by their name
using ParameterSet = std::unordered_set<rclcpp::Parameter, detail::params_comparator_t>;

/**
 * @brief Declares a ROS 2 parameter if this is not already declared
 * and returns the current value of the parameter.
 * @note It's always recommended to use this utility when declaring a parameter,
 * as the standard API will throw an exception if this was already declared.
 * This situation is common when dealing with lifecycle nodes that can be configured
 * multiple times.
 * @param node_parameters ROS 2 node parameter interface that will own the parameter
 * @param param_name name of the parameter
 * @param default_value default value of the parameter
 * @param parameter_descriptor optional description of the parameter
 * @return current value of the parameter
 */
rclcpp::ParameterValue declare_parameter_if_not_declared(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor());

}  // namespace wombat_core
