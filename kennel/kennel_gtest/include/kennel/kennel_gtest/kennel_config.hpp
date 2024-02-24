// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace kennel
{

static constexpr auto DEFAULT_ROBOT_NAME = "my_robot";
static constexpr auto DEFAULT_ROBOT_NODE_NAME = "/my_robot/robot_sim";
static constexpr auto KENNEL_NAME = "/kennel";

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
  add_robot(const std::string & robot_name = DEFAULT_ROBOT_NAME);

  KennelParamsConfig &
  set_robot_pose(
    const std::vector<double> & pose_2d,
    const std::string & robot_name = DEFAULT_ROBOT_NODE_NAME);

  KennelParamsConfig &
  add_lidar_slam_positioner_to_robot(
    const std::string & robot_name = DEFAULT_ROBOT_NODE_NAME,
    const std::string & plugin_name = "map");

  KennelParamsConfig &
  add_local_slam_positioner_to_robot(
    const std::string & robot_name = DEFAULT_ROBOT_NODE_NAME,
    const std::string & plugin_name = "map");

  KennelParamsConfig &
  add_bumper_to_robot(const std::string & robot_name = DEFAULT_ROBOT_NODE_NAME);

  KennelParamsConfig &
  add_lidar2d_to_robot(
    const std::string & robot_name = DEFAULT_ROBOT_NODE_NAME,
    const std::optional<double> & range_max = std::nullopt);

  rclcpp::ParameterMap get() const;

private:
  void add_positioner_to_robot(
    const std::string & robot_name,
    const std::string & plugin_name,
    const std::string & positioner_type);

  void add_sensor_to_robot(
    const std::string & robot_name,
    const std::string & plugin_name,
    const std::string & sensor_type);

  std::vector<std::string> get_robot_names() const;

  bool has_robot(const std::string & robot_name) const;

  void print_parameters_map() const;

  rclcpp::ParameterMap m_parameter_map;
};

}  // namespace kennel
