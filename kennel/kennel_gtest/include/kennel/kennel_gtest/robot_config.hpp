// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace kennel
{
namespace config
{

bool add_bumper_to_robot_params(
  rclcpp::ParameterMap & parameter_map,
  const std::string & fully_qualified_node_name = "/my_robot/robot_sim");

bool add_lidar2d_to_robot_params(
  rclcpp::ParameterMap & parameter_map,
  const std::string & fully_qualified_node_name = "/my_robot/robot_sim");

}  // namespace config
}  // namespace kennel
