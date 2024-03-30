// Copyright 2024 Soragna Alberto.

#pragma once

#include <functional>
#include <optional>

#include "nav_msgs/msg/map_meta_data.hpp"

#include "wombat_core/grid/coordinates.hpp"

namespace wombat_core
{

/**
 * @brief Grid neighbors iterator.
 * Execute the predicate on all the grid neighbors of the provided
 * index, or until told to stop.
 * The neighbors will be processed in an arbitrary order.
 * @param index grid index where to look for neighbors
 * @param grid_width width (x) dimension of the grid
 * @param grid_height height (y) dimension of the grid
 * @param predicate function to run on the neighbors.
 * It should return true if we want to stop early.
 * @param eight_connected whether we should examine 4 or 8 connected
 * neighbors
 */
void for_each_grid_neighbor(
  grid_index_t index,
  unsigned int grid_width,
  unsigned int grid_height,
  const std::function<bool(grid_index_t)> & predicate,
  bool eight_connected = false);

}  // namespace wombat_core
