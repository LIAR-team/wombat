// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#pragma once

#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace wombat_ops
{

/**
 * @brief Computes a point in the world frame corresponding to the
 * center of map cell at the provided index.
 */
geometry_msgs::msg::Point index_to_world(
  unsigned int index,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap);

/**
 * @brief Computes the index of the map cell with the center that is
 * the closest to the provided world point.
 */
unsigned int world_to_index(
  const geometry_msgs::msg::Point & world,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap);

}  // namespace wombat_ops
