// Copyright 2024 Soragna Alberto.

#pragma once

#include "geometry_msgs/msg/pose.hpp"

namespace kennel
{

bool pose_2d_equal(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2);

}  // namespace kennel
