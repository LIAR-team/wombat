// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace wombat_core
{

namespace occupancy
{
  constexpr char FREE = 0;
  constexpr char INSCRIBED_OBS = 99;
  constexpr char LETHAL_OBS = 100;
  constexpr char UNKNOWN = -1;
}  // namespace occupancy

/**
 * @brief Writes an occupancy grid message from a costmap object.
 * The function will use thread-safe access to the costmap, locking it
 * while using it.
 * @note This function will not touch the header field of the grid msg
 * @param costmap costmap to read from
 * @param grid occupancy grid msgs to write into
 */
void costmap_to_occupancy_grid_values(
  nav2_costmap_2d::Costmap2D & costmap,
  nav_msgs::msg::OccupancyGrid & grid);

}  // namespace wombat_core
