// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <functional>
#include <limits>
#include <optional>

#include "nav_msgs/msg/map_meta_data.hpp"

#include "wombat_core/cpp/types.hpp"
#include "wombat_core/math/grid/coordinates.hpp"

namespace wombat_core
{

/**
 * @brief Raytrace iterator. This function draws a line between two
 * grid points and runs an evaluation
 * function on each grid cell that touches the line.
 * It requires that both grid points are within the map.
 * Returns the index of the first cell that satisfies the evaluation.
 * @param from_grid grid point where to start raytracing
 * @param to_grid grid point where to end raytracing
 * @param map_info information about the grid
 * @param predicate function to run on every grid point on the line.
 * It should return true if we want to stop early.
 * @param length_range start raytracing from this min distance. If 0.0, then
 * it will start from the provided "from" point and stop raytracing after the max
 * distance or when the "to" point is reached, whatever occurs first.
 * @return std::optional<grid_index_t> index of a grid cell that satisfies the
 * evaluation function or std::nullopt if none is found or an error occurs.
 */
std::optional<grid_index_t> find_if_raytrace(
  const grid_coord_t & from_grid,
  const grid_coord_t & to_grid,
  const nav_msgs::msg::MapMetaData & map_info,
  const std::function<bool(grid_index_t)> & predicate,
  const double_range_t & length_range = {0.0, std::numeric_limits<double>::max()});

/**
 * @brief Project a grid coordinate along a direction until
 * it intersects with the grid boundaries
 * @param from_grid the grid coordinates where to start from
 * @param map_info information about the grid
 * @param angle angle to project along
 * @return std::optional<grid_coord_t> intersection coordinate or std::nullopt
 * if something went wrong
 */
std::optional<grid_coord_t> project_to_grid_boundary(
  const grid_coord_t & from_grid,
  const nav_msgs::msg::MapMetaData & map_info,
  double angle);

}  // namespace wombat_core
