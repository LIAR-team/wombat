// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <iostream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel_gtest/kennel_config.hpp"
#include "wombat_core/ros2/parameters.hpp"

namespace kennel
{

rclcpp::ParameterMap KennelParamsConfig::get() const
{
  this->print_parameters_map();
  return m_parameter_map;
}

KennelParamsConfig &
KennelParamsConfig::set_map_yaml_filename(const std::string & yaml_file)
{
  bool success = wombat_core::write_parameter_map(
    m_parameter_map,
    KENNEL_NAME,
    "map_yaml_filename",
    rclcpp::ParameterValue(yaml_file),
    true);
  if (!success) {
    throw std::runtime_error("Failed to write yaml filename");
  }

  std::string map_topic_name = "";
  if (!yaml_file.empty()) {
    map_topic_name = "/ground_truth_map";
  }
  const auto & robot_names = this->get_robot_names();
  for (const auto & name : robot_names) {
    std::string full_name = "/" + name + "/robot_sim";
    success = wombat_core::write_parameter_map(
      m_parameter_map,
      full_name,
      "mobile_base.ground_truth.map_topic_name",
      rclcpp::ParameterValue(map_topic_name));
    if (!success) {
      throw std::runtime_error("Failed to write robot map topic name");
    }
  }

  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_robot(const std::string & robot_name)
{
  if (m_parameter_map.count(robot_name) != 0) {
    throw std::runtime_error("Can't add robot because it already exists");
  }

  bool success = wombat_core::append_parameter_map(
    m_parameter_map,
    KENNEL_NAME,
    "robots",
    rclcpp::ParameterValue(std::vector<std::string>({robot_name})));
  if (!success) {
    throw std::runtime_error("Failed to add robot");
  }

  return *this;
}

KennelParamsConfig &
KennelParamsConfig::set_robot_pose(
  const std::vector<double> & pose_2d,
  const std::string & robot_name)
{
  if (!this->has_robot(robot_name)) {
    throw std::runtime_error("Can't set pose for not existing robot");
  }
  bool success = wombat_core::write_parameter_map(
    m_parameter_map,
    robot_name,
    "mobile_base.start_pose",
    rclcpp::ParameterValue(pose_2d),
    true);
  if (!success) {
    throw std::runtime_error("Failed to write robot start pose");
  }

  return *this;
}

void KennelParamsConfig::add_positioner_to_robot(
  const std::string & robot_name,
  const std::string & plugin_name,
  const std::string & positioner_type)
{
  if (!this->has_robot(robot_name)) {
    throw std::runtime_error("Can't add positioner for not existing " + robot_name);
  }

  bool success = wombat_core::append_parameter_map(
    m_parameter_map,
    robot_name,
    "mobile_base.positioners",
    rclcpp::ParameterValue(std::vector<std::string>({plugin_name})));
  if (!success) {
    throw std::runtime_error("Failed to append positioner");
  }
  success = wombat_core::write_parameter_map(
    m_parameter_map,
    robot_name,
    "mobile_base." + plugin_name + ".plugin_type",
    rclcpp::ParameterValue(positioner_type),
    true);
  if (!success) {
    throw std::runtime_error("Failed to write positioner plugin type");
  }
}

KennelParamsConfig &
KennelParamsConfig::add_lidar_slam_positioner_to_robot(
  const std::string & robot_name,
  const std::string & plugin_name)
{
  this->add_positioner_to_robot(robot_name, plugin_name, "kennel::LidarSLAMPositioner");
  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_local_slam_positioner_to_robot(
  const std::string & robot_name,
  const std::string & plugin_name)
{
  this->add_positioner_to_robot(robot_name, plugin_name, "kennel::LocalSLAMPositioner");
  return *this;
}

void KennelParamsConfig::add_sensor_to_robot(
  const std::string & robot_name,
  const std::string & plugin_name,
  const std::string & sensor_type)
{
  if (!this->has_robot(robot_name)) {
    throw std::runtime_error("Can't add sensor for not existing " + robot_name);
  }

  bool success = wombat_core::append_parameter_map(
    m_parameter_map,
    robot_name,
    "sensors",
    rclcpp::ParameterValue(std::vector<std::string>({plugin_name})));
  if (!success) {
    throw std::runtime_error("Failed to append sensor");
  }
  success = wombat_core::write_parameter_map(
    m_parameter_map,
    robot_name,
    plugin_name + ".plugin_type",
    rclcpp::ParameterValue(sensor_type),
    true);
  if (!success) {
    throw std::runtime_error("Failed to write sensor plugin type");
  }
}

KennelParamsConfig &
KennelParamsConfig::add_bumper_to_robot(
  const std::string & robot_name)
{
  this->add_sensor_to_robot(robot_name, "bumper", "kennel::Bumper");
  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_lidar2d_to_robot(
  const std::string & robot_name,
  const std::optional<double> & range_max)
{
  this->add_sensor_to_robot(robot_name, "base_scan", "kennel::Lidar2D");

  if (range_max) {
    bool success = wombat_core::write_parameter_map(
      m_parameter_map,
      robot_name,
      "base_scan.range_max",
      rclcpp::ParameterValue(*range_max),
      true);
    if (!success) {
      throw std::runtime_error("Failed to write lidar range_max type");
    }
  }

  return *this;
}

std::vector<std::string> KennelParamsConfig::get_robot_names() const
{
  std::vector<std::string> robot_names;
  auto maybe_robots_param = wombat_core::get_parameter_for_node(
    "robots",
    m_parameter_map,
    KENNEL_NAME);
  if (!maybe_robots_param) {
    return std::vector<std::string>();
  }
  return maybe_robots_param->as_string_array();
}

bool KennelParamsConfig::has_robot(const std::string & robot_name) const
{
  std::string robot_namespace = robot_name;
  static const std::string FULL_NAME_PREFIX = "/";
  static const std::string FULL_NAME_SUFFIX = "/robot_sim";
  const auto & this_prefix = robot_namespace.substr(0, FULL_NAME_PREFIX.length());
  if (this_prefix == FULL_NAME_PREFIX) {
    robot_namespace = robot_namespace.substr(FULL_NAME_PREFIX.length());
  }
  const auto & this_suffix =
    robot_namespace.substr(robot_namespace.length() - FULL_NAME_SUFFIX.length(), robot_namespace.length());
  if (this_suffix == FULL_NAME_SUFFIX) {
    robot_namespace = robot_namespace.substr(0, robot_namespace.length() - FULL_NAME_SUFFIX.length());
  }

  const auto & robot_names = this->get_robot_names();
  if (robot_names.empty()) {
    return false;
  }
  auto robot_name_it = std::find(robot_names.begin(), robot_names.end(), robot_namespace);
  return robot_name_it != robot_names.end();
}

void KennelParamsConfig::print_parameters_map() const
{
  std::cout << "----- Kennel parameters map ----- " << std::endl;
  for (const auto & node_map : m_parameter_map) {
    std::cout << "Node: " << node_map.first << ":" << std::endl;
    for (const auto & param : node_map.second) {
      std::cout << "  - " << param.get_name() << " '" << param.value_to_string() << "'" << std::endl;
    }
  }
}

}  // namespace kennel
