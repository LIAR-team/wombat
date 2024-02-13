// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "wombat_core/math/grid/coordinates.hpp"
#include "wombat_core/math/grid/neighbors.hpp"

#include <iostream>

namespace wombat_core
{

void for_each_grid_neighbor(
  grid_index_t index,
  unsigned int grid_width,
  unsigned int grid_height,
  const std::function<bool(grid_index_t)> & predicate,
  bool eight_connected)
{
  // If the cell index is outside the map, no neighbors to process
  // Note: this check also ensures that the grid dimensions are both non-zero
  if (index >= grid_width * grid_height) {
    return;
  }

  if (index % grid_width > 0) {
    if (predicate(index - 1)) {
      return;
    }
    if (eight_connected && index >= grid_width) {
      if (predicate(index - 1 - grid_width)) {
        return;
      }
    }
    if (eight_connected && index < grid_width * (grid_height - 1)) {
      if (predicate(index - 1 + grid_width)) {
        return;
      }
    }
  }

  if (index % grid_width < grid_width - 1) {
    if (predicate(index + 1)) {
      return;
    }
    if (eight_connected && index >= grid_width) {
      if (predicate(index + 1 - grid_width)) {
        return;
      }
    }
    if (eight_connected && index < grid_width * (grid_height - 1)) {
      if (predicate(index + 1 + grid_width)) {
        return;
      }
    }
  }

  if (index >= grid_width) {
    if (predicate(index - grid_width)) {
      return;
    }
  }

  if (index < grid_width * (grid_height - 1)) {
    if (predicate(index + grid_width)) {
      return;
    }
  }
}

}  // namespace wombat_core
