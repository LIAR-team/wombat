// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace kennel
{

static constexpr auto DEFAULT_ROBOT_NAMESPACE = "my_robot";
static constexpr auto DEFAULT_ROBOT_NODE = "/my_robot/robot_sim";
static constexpr auto KENNEL_NODE = "/kennel";

using NamedParams = std::map<std::string, rclcpp::ParameterValue>;

class KennelParamsConfig
{
public:
  KennelParamsConfig() = default;

  explicit KennelParamsConfig(const std::string & yaml_file_path)
  {
    m_parameter_map = rclcpp::parameter_map_from_yaml_file(yaml_file_path);
  }

  explicit KennelParamsConfig(rclcpp::ParameterMap parameter_map)
  : m_parameter_map(std::move(parameter_map))
  {}

  KennelParamsConfig &
  set_map_yaml_filename(const std::string & yaml_file);

  KennelParamsConfig &
  add_robot(const std::string & robot_name = DEFAULT_ROBOT_NAMESPACE);

  KennelParamsConfig &
  set_robot_pose(
    const std::vector<double> & pose_2d,
    const std::string & robot_name = DEFAULT_ROBOT_NODE);

  KennelParamsConfig &
  set_robot_radius(
    double robot_radius,
    const std::string & robot_name = DEFAULT_ROBOT_NODE);

  KennelParamsConfig &
  add_stub_positioner_to_robot(
    const std::string & robot_name = DEFAULT_ROBOT_NODE,
    const NamedParams & plugin_params = NamedParams(),
    const std::string & plugin_name = "map");

  KennelParamsConfig &
  add_lidar_slam_positioner_to_robot(
    const std::string & robot_name = DEFAULT_ROBOT_NODE,
    const NamedParams & plugin_params = NamedParams(),
    const std::string & plugin_name = "map");

  KennelParamsConfig &
  add_local_slam_positioner_to_robot(
    const std::string & robot_name = DEFAULT_ROBOT_NODE,
    const NamedParams & plugin_params = NamedParams(),
    const std::string & plugin_name = "map");

  KennelParamsConfig &
  add_bumper_to_robot(
    const std::string & robot_name = DEFAULT_ROBOT_NODE,
    const NamedParams & plugin_params = NamedParams(),
    const std::string & plugin_name = "bumper");

  KennelParamsConfig &
  add_lidar2d_to_robot(
    const std::string & robot_name = DEFAULT_ROBOT_NODE,
    const NamedParams & plugin_params = NamedParams(),
    const std::string & plugin_name = "base_scan");

  rclcpp::ParameterMap get() const;

private:
  struct plugin_description_t
  {
    std::string name;
    std::string prefix;
    std::string list_param_name;
    std::string type;

    std::string full_param_name(const std::string & rel_name) const
    {
      const std::string prefix_element = prefix.empty() ? "" : prefix + ".";
      return prefix_element + name + "." + rel_name;
    }
  };

  void add_plugin_to_robot(
    const std::string & robot_name,
    const plugin_description_t & plugin,
    const NamedParams & plugin_params);

  std::vector<std::string> get_robot_names() const;

  bool has_robot(const std::string & robot_name) const;

  void write_parameter_for_robot(
    const rclcpp::Parameter & param,
    const std::string & robot_name);

  void write_parameter_for_node(
    const rclcpp::Parameter & param,
    const std::string & node_name);

  void print_parameters_map() const;

  rclcpp::ParameterMap m_parameter_map;
};

}  // namespace kennel
