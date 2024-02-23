// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace kennel
{

static constexpr auto DEFAULT_ROBOT_NAME = "/my_robot/robot_sim";

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
  add_robot(const std::string & robot_name);

  KennelParamsConfig &
  add_bumper_to_robot(const std::string & robot_name = DEFAULT_ROBOT_NAME);

  KennelParamsConfig &
  add_lidar2d_to_robot(const std::string & robot_name = DEFAULT_ROBOT_NAME);

  rclcpp::ParameterMap get() const;

private:
  rclcpp::ParameterMap m_parameter_map;
};

}  // namespace kennel
