// Copyright 2024 Soragna Alberto.

#pragma once

#include "geometry_msgs/msg/pose.hpp"

namespace wombat_core
{

template<typename T>
T linear_interpolation(const T & start, const T & end, double factor)
{
  return start + static_cast<T>(factor * static_cast<double>(end - start));
}

geometry_msgs::msg::Pose pose_interpolation(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & end,
  double factor);

}  // namespace wombat_core
