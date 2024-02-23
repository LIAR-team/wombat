// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel_gtest/robot_config.hpp"
#include "wombat_core/ros2/parameters.hpp"

namespace kennel
{
namespace config
{

bool add_bumper_to_robot_params(
  rclcpp::ParameterMap & parameter_map,
  const std::string & fully_qualified_node_name)
{
  bool success = false;
  success = wombat_core::append_parameter_map(
    parameter_map,
    fully_qualified_node_name,
    "sensors",
    rclcpp::ParameterValue(std::vector<std::string>({"bumper"})));
  if (!success) {
    return false;
  }

  success = wombat_core::write_parameter_map(
    parameter_map,
    fully_qualified_node_name,
    "bumper.plugin_type",
    rclcpp::ParameterValue("kennel::Bumper"));

  return success;
}

bool add_lidar2d_to_robot_params(
  rclcpp::ParameterMap & parameter_map,
  const std::string & fully_qualified_node_name)
{
  bool success = false;
  success = wombat_core::append_parameter_map(
    parameter_map,
    fully_qualified_node_name,
    "sensors",
    rclcpp::ParameterValue(std::vector<std::string>({"lidar2d"})));
  if (!success) {
    return false;
  }

  success = wombat_core::write_parameter_map(
    parameter_map,
    fully_qualified_node_name,
    "lidar2d.plugin_type",
    rclcpp::ParameterValue("kennel::Lidar2D"));

  success = wombat_core::write_parameter_map(
    parameter_map,
    fully_qualified_node_name,
    "lidar2d.topic_name",
    rclcpp::ParameterValue("base_scan"));

  return success;
}

}  // namespace config
}  // namespace kennel
