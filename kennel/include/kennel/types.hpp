// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace kennel
{

struct LocalizationData
{
  geometry_msgs::msg::TransformStamped robot_pose;
  nav_msgs::msg::OccupancyGrid map;
};

}  // namespace kennel
