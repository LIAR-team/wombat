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
  this->write_parameter_for_node(
    rclcpp::Parameter("map_yaml_filename", rclcpp::ParameterValue(yaml_file)),
    KENNEL_NAME);

  std::string map_topic_name = "";
  if (!yaml_file.empty()) {
    map_topic_name = "/ground_truth_map";
  }

  const auto & robot_names = this->get_robot_names();
  for (const auto & name : robot_names) {
    std::string full_name = "/" + name + "/robot_sim";
    this->write_parameter_for_robot(
      rclcpp::Parameter("mobile_base.ground_truth.map_topic_name", rclcpp::ParameterValue(map_topic_name)),
      full_name);
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
KennelParamsConfig::set_robot_radius(
  double robot_radius,
  const std::string & robot_name)
{
  this->write_parameter_for_robot(
    rclcpp::Parameter("mobile_base.robot_radius", rclcpp::ParameterValue(robot_radius)),
    robot_name);
  return *this;
}

KennelParamsConfig &
KennelParamsConfig::set_robot_pose(
  const std::vector<double> & pose_2d,
  const std::string & robot_name)
{
  this->write_parameter_for_robot(
    rclcpp::Parameter("mobile_base.start_pose", rclcpp::ParameterValue(pose_2d)),
    robot_name);
  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_lidar_slam_positioner_to_robot(
  const std::string & robot_name,
  const NamedParams & plugin_params,
  const std::string & plugin_name)
{
  plugin_description_t plugin;
  plugin.name = plugin_name;
  plugin.prefix = "mobile_base";
  plugin.list_param_name = "positioners";
  plugin.type = "kennel::LidarSLAMPositioner";

  this->add_plugin_to_robot(
    robot_name,
    plugin,
    plugin_params);
  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_local_slam_positioner_to_robot(
  const std::string & robot_name,
  const NamedParams & plugin_params,
  const std::string & plugin_name)
{
  plugin_description_t plugin;
  plugin.name = plugin_name;
  plugin.prefix = "mobile_base";
  plugin.list_param_name = "positioners";
  plugin.type = "kennel::LocalSLAMPositioner";

  this->add_plugin_to_robot(
    robot_name,
    plugin,
    plugin_params);
  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_bumper_to_robot(
  const std::string & robot_name,
  const NamedParams & plugin_params,
  const std::string & plugin_name)
{
  plugin_description_t plugin;
  plugin.name = plugin_name;
  plugin.prefix = "sensors_manager";
  plugin.list_param_name = "sensors";
  plugin.type = "kennel::Bumper";

  this->add_plugin_to_robot(
    robot_name,
    plugin,
    plugin_params);
  return *this;
}

KennelParamsConfig &
KennelParamsConfig::add_lidar2d_to_robot(
  const std::string & robot_name,
  const NamedParams & plugin_params,
  const std::string & plugin_name)
{
  plugin_description_t plugin;
  plugin.name = plugin_name;
  plugin.prefix = "sensors_manager";
  plugin.list_param_name = "sensors";
  plugin.type = "kennel::Lidar2D";

  this->add_plugin_to_robot(
    robot_name,
    plugin,
    plugin_params);
  return *this;
}

void KennelParamsConfig::add_plugin_to_robot(
  const std::string & robot_name,
  const plugin_description_t & plugin,
  const NamedParams & plugin_params)
{
  if (!this->has_robot(robot_name)) {
    std::stringstream error_text;
    error_text << "Can't add plugin '" << plugin.name << "'";
    error_text << " for not existing robot '" << robot_name << "'";
    throw std::runtime_error(error_text.str());
  }

  const std::string full_plugin_list_param_name = plugin.prefix + "." + plugin.list_param_name;
  bool success = wombat_core::append_parameter_map(
    m_parameter_map,
    robot_name,
    full_plugin_list_param_name,
    rclcpp::ParameterValue(std::vector<std::string>({plugin.name})));
  if (!success) {
    std::stringstream error_text;
    error_text << "Failed to append plugin '" << plugin.name << "' to list '" << full_plugin_list_param_name << "'";
    error_text << " for robot '" << robot_name << "'";
    throw std::runtime_error(error_text.str());
  }

  static constexpr auto PLUGIN_TYPE_P_NAME = "plugin_type";
  auto extended_plugin_params = plugin_params;
  const auto plugin_type_map_entry = std::make_pair(PLUGIN_TYPE_P_NAME, rclcpp::ParameterValue(plugin.type));
  const auto insert_result = extended_plugin_params.insert(plugin_type_map_entry);
  if (!insert_result.second) {
    std::stringstream error_text;
    error_text << "Params for plugin '" << plugin.name << "' can't include '" << PLUGIN_TYPE_P_NAME << "' parameter";
    error_text << " for robot '" << robot_name << "'";
    throw std::runtime_error(error_text.str());
  }

  for (const auto & [p_name, p_value] : extended_plugin_params) {
    const std::string full_plugin_param_name = plugin.full_param_name(p_name);
    this->write_parameter_for_robot(
      rclcpp::Parameter(full_plugin_param_name, p_value),
      robot_name);
  }
}

void KennelParamsConfig::write_parameter_for_robot(
  const rclcpp::Parameter & param,
  const std::string & robot_name)
{
  if (!this->has_robot(robot_name)) {
    std::stringstream error_text;
    error_text << "Can't write parameter '" << param.get_name() << "'";
    error_text << " for not existing robot '" << robot_name << "'";
    throw std::runtime_error(error_text.str());
  }
  this->write_parameter_for_node(param, robot_name);
}

void KennelParamsConfig::write_parameter_for_node(
  const rclcpp::Parameter & param,
  const std::string & node_name)
{
  bool success = wombat_core::write_parameter_map(
    m_parameter_map,
    node_name,
    param.get_name(),
    param.get_parameter_value(),
    true);
  if (!success) {
    std::stringstream error_text;
    error_text << "Failed to write param '" << param.get_name() << "' with value '" << param.value_to_string() << "'";
    error_text << " for node '" << node_name << "'";
    throw std::runtime_error(error_text.str());
  }
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
  std::stringstream params_txt;
  params_txt << "----- Kennel parameters map -----\n";
  for (const auto & node_map : m_parameter_map) {
    params_txt << node_map.first << ":\n";
    for (const auto & param : node_map.second) {
      params_txt << "  - " << param.get_name() << " '" << param.value_to_string() << "'\n";
    }
  }
  params_txt << "----- ----- ----- ";
  std::cout << params_txt.str() << std::endl;
}

}  // namespace kennel
