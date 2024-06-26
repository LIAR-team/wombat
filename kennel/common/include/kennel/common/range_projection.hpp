// Copyright 2024 Soragna Alberto.

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "wombat_core/grid/types.hpp"

namespace kennel
{

/**
 * @brief Compute laser scanner ranges on a map by projecting beams from the
 * provided pose.
 * The output vector will either have a number of elements matching num_bins or
 * it will be empty if something goes wrong.
 * @param map the map to compute ranges on
 * @param laser_pose start pose of the ranges
 * @param num_bins number of ranges
 * @param angle_range minimum and maximum angle in laser reference frame
 * @param distance_range minimum and maximum range distance
 * @return std::vector<wombat_core::grid_coord_t> computed ranges
 */
std::vector<wombat_core::grid_coord_t> compute_laser_projections(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & laser_pose,
  size_t num_bins,
  const std::pair<float, float> & angle_range,
  const std::pair<float, float> & distance_range);

}  // namespace kennel
