// Copyright 2024 Soragna Alberto.

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace kennel
{

namespace names
{

static constexpr auto DEFAULT_ROBOT_NAMESPACE = "my_robot";
static constexpr auto DEFAULT_ROBOT_NODE = "/my_robot/robot_sim";
static constexpr auto KENNEL_NODE = "/kennel";

static constexpr auto POSITIONER_LIDAR_SLAM = "kennel::LidarSLAMPositioner";
static constexpr auto POSITIONER_LOCAL_SLAM = "kennel::LocalSLAMPositioner";
static constexpr auto POSITIONER_STUB = "kennel::StubPositioner";

static constexpr auto SENSOR_BUMPER = "kennel::Bumper";
static constexpr auto SENSOR_LIDAR2D = "kennel::Lidar2D";

}  // namespace names

using named_params_t = std::map<std::string, rclcpp::ParameterValue>;

struct named_plugin_t
{
  std::string type;
  std::string name;
};

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
  add_robot(const std::string & robot_name = names::DEFAULT_ROBOT_NAMESPACE);

  KennelParamsConfig &
  set_robot_pose(
    const std::vector<double> & pose_2d,
    const std::string & robot_name = names::DEFAULT_ROBOT_NODE);

  KennelParamsConfig &
  set_robot_radius(
    double robot_radius,
    const std::string & robot_name = names::DEFAULT_ROBOT_NODE);

  KennelParamsConfig &
  add_positioner_plugin_to_robot(
    const named_plugin_t & plugin,
    const named_params_t & plugin_params = named_params_t(),
    const std::string & robot_name = names::DEFAULT_ROBOT_NODE);

  KennelParamsConfig &
  add_sensor_plugin_to_robot(
    const named_plugin_t & plugin,
    const named_params_t & plugin_params = named_params_t(),
    const std::string & robot_name = names::DEFAULT_ROBOT_NODE);

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
    const named_params_t & plugin_params);

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
