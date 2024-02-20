// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <Eigen/Core>

namespace wombat_core
{

/** @brief The index to access elements on a grid */
using grid_index_t = size_t;

/** @brief The coordinates of a cell on a grid */
struct grid_coord_t
{
  grid_index_t x;
  grid_index_t y;
};

using Coord2D = Eigen::Array2i;
using Size = Eigen::Array2i;

}  // namespace wombat_core
