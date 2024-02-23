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

rclcpp::ParameterMap KennelParamsConfig::get() const
{
  return m_parameter_map;
}

KennelParamsConfig &
KennelParamsConfig::set_map_yaml_filename(const std::string & yaml_file)
{
  bool success = wombat_core::write_parameter_map(
    m_parameter_map,
    "/kennel",
    "map_yaml_filename",
    rclcpp::ParameterValue(yaml_file),
    true);
  if (!success) {
    throw std::runtime_error("Failed to write yaml filename");
  }

  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_robot(const std::string & robot_name)
{
  (void)robot_name;
  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_bumper_to_robot(const std::string & robot_name)
{
  bool success = wombat_core::append_parameter_map(
    m_parameter_map,
    robot_name,
    "sensors",
    rclcpp::ParameterValue(std::vector<std::string>({"bumper"})));
  if (!success) {
    throw std::runtime_error("Failed to append bumper sensor");
  }

  success = wombat_core::write_parameter_map(
    m_parameter_map,
    robot_name,
    "bumper.plugin_type",
    rclcpp::ParameterValue("kennel::Bumper"),
    true);
  if (!success) {
    throw std::runtime_error("Failed to write bumper plugin type");
  }

  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_lidar2d_to_robot(const std::string & robot_name)
{
  bool success = false;
  success = wombat_core::append_parameter_map(
    m_parameter_map,
    robot_name,
    "sensors",
    rclcpp::ParameterValue(std::vector<std::string>({"base_scan"})));
  if (!success) {
    throw std::runtime_error("Failed to append lidar sensor");
  }

  success = wombat_core::write_parameter_map(
    m_parameter_map,
    robot_name,
    "base_scan.plugin_type",
    rclcpp::ParameterValue("kennel::Lidar2D"),
    true);
  if (!success) {
    throw std::runtime_error("Failed to write lidar plugin type");
  }

  return *this;
}

}  // namespace kennel
