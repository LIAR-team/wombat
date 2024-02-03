// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <functional>
#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

namespace wombat_core
{

/** @brief The coordinates of a cell on a grid */
struct grid_coord_t
{
  unsigned int x;
  unsigned int y;
};

/** @brief The index to access elements on a grid */
using grid_index_t = size_t;

/**
 * @brief Converts grid coordinates into an index
 * that can be used to reference values when a grid is represented
 * as a flat (1-dimensional) container.
 * @param grid_coord the coordinate to convert
 * @param map_info information about the grid
 * @return std::optional<grid_index_t> index or nullopt if failed
 */
std::optional<grid_index_t> grid_coord_to_index(
  const grid_coord_t & grid_coord,
  const nav_msgs::msg::MapMetaData & map_info);

/**
 * @brief Converts a world point into its corresponding grid
 * coordinate
 * @param world_pt the world point to convert
 * @param map_info information about the grid
 * @return std::optional<grid_coord_t> grid coord or nullopt if failed
 */
std::optional<grid_coord_t> world_pt_to_grid_coord(
  const geometry_msgs::msg::Point & world_pt,
  const nav_msgs::msg::MapMetaData & map_info);

/**
 * @brief Converts a world point into the index used to access
 * the corresponding grid coordinate
 * @param world_pt the world point to convert
 * @param map_info information about the grid
 * @return std::optional<grid_index_t> grid index or nullopt if failed
 */
std::optional<grid_index_t> world_pt_to_grid_index(
  const geometry_msgs::msg::Point & world_pt,
  const nav_msgs::msg::MapMetaData & map_info);

/**
 * @brief Grid-line iterator. This function draws a line between two
 * world points, discretizes it to a grid, and runs an evaluation
 * function on each grid cell that touches the line.
 * Returns the index of the first cell that satisfies the evaluation.
 * @param from_world world point where the line starts
 * @param to_world world point where the line ends
 * @param map_info information about the grid
 * @param eval_func function to run on every grid point on the line
 * @return std::optional<grid_index_t> index of a grid cell that satisfies the
 * evaluation function or std::nullopt if none is found
 */
std::optional<grid_index_t> find_grid_coord_on_line(
  const geometry_msgs::msg::Point & from_world,
  const geometry_msgs::msg::Point & to_world,
  const nav_msgs::msg::MapMetaData & map_info,
  const std::function<bool(grid_index_t)> & eval_func);

}  // namespace wombat_core
