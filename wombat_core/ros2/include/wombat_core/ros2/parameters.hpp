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

/**
 * @brief Updates a parameter map setting a new value for a parameter
 * @param parameter_map the map to update
 * @param fully_qualified_node_name the fully qualified name of the node where to
 * write the parameter
 * @param param_name the name of the parameter
 * @param param_value the value of the parameter
 * @param allow_override whether we should override a value if this parameter is already
 * set
 * @return true if the parameter has been written
 */
bool update_parameter_map(
  rclcpp::ParameterMap & parameter_map,
  const std::string & fully_qualified_node_name,
  const std::string & param_name,
  const rclcpp::ParameterValue & param_value,
  bool allow_override = true);

/**
 * @brief Get a parameter from a parameter map structure
 * @note This is not a simple name lookup, it will also take
 * into account regexes in the fqn
 * @param param_name name of the parameter to read
 * @param parameter_map map where to look for the parameter
 * @param fully_qualified_node_name name of the node where to read the parameter
 * @return std::optional<rclcpp::Parameter> the parameter if found or nullopt
 */
std::optional<rclcpp::Parameter> get_parameter_for_node(
  const std::string & param_name,
  const rclcpp::ParameterMap & parameter_map,
  const std::string & fully_qualified_name);

/**
 * @brief Set the parameters from a parameter map into a ROS 2 node
 * identified via its node interfaces
 * @param parameter_map the map to load parameters from
 * @param node_base_ifc the base interface of the node
 * @param node_parameters_ifc the parameters interface of the node
 * @return true if all parameters loaded have been correctly set
 */
bool set_parameters_from_map(
  const rclcpp::ParameterMap & parameter_map,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ifc,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_ifc);

}  // namespace wombat_core
