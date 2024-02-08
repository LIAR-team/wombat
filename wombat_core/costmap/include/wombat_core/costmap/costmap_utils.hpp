// Copyright 2021-2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace wombat_core
{

/**
 * @brief Compute the point in the world frame corresponding to the center
 * of the costamp cell identified by the provided index.
 * @param index index of the costmap cell
 * @param costmap costmap
 * @return geometry_msgs::msg::Point world frame point
 */
geometry_msgs::msg::Point index_to_world(
  unsigned int index,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap);

/**
 * @brief Compute the index of the costmap cell that is closest to the world frame point
 * provided as input.
 * @param world world frame point
 * @param costmap costmap
 * @return unsigned int index of the costmap cell
 */
unsigned int world_to_index(
  const geometry_msgs::msg::Point & world,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap);

}  // namespace wombat_core
