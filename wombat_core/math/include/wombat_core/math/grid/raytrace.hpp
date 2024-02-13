// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <functional>
#include <optional>

#include "nav_msgs/msg/map_meta_data.hpp"

#include "wombat_core/math/grid/coordinates.hpp"

namespace wombat_core
{

/**
 * @brief Raytrace iterator. This function draws a line between two
 * grid points and runs an evaluation
 * function on each grid cell that touches the line.
 * Returns the index of the first cell that satisfies the evaluation.
 * @param from_grid grid point where to start raytracing
 * @param to_grid grid point where to end raytracing
 * @param map_info information about the grid
 * @param predicate function to run on every grid point on the line.
 * It should return true if we want to stop early.
 * @return std::optional<grid_index_t> index of a grid cell that satisfies the
 * evaluation function or std::nullopt if none is found
 */
std::optional<grid_index_t> find_if_raytrace(
  const grid_coord_t & from_grid,
  const grid_coord_t & to_grid,
  const nav_msgs::msg::MapMetaData & map_info,
  const std::function<bool(grid_index_t)> & predicate);

}  // namespace wombat_core
