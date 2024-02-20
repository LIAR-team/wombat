// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <optional>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "wombat_core/grid/types.hpp"

namespace wombat_core
{

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
 * @brief Converts grid vector index (used to reference values when a grid
 * is represented as a flat container) into a grid coordinate.
 * @param grid_index the index to convert
 * @param map_info information about the grid
 * @return std::optional<grid_coord_t> grid coordinate or nullopt if failed
 */
std::optional<grid_coord_t> grid_index_to_coord(
  const grid_index_t & grid_index,
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
 * @brief Converts a grid coordinate into the corresponding world point.
 * @param grid_coord_t the grid coordinate to convert
 * @param map_info information about the grid
 * @return world point or nullopt if failed
 */
std::optional<geometry_msgs::msg::Point> grid_coord_to_world_pt(
  const grid_coord_t & grid_coord,
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
 * @brief Converts a grid index into the corresponding world point
 * @param grid_index the grid index to convert
 * @param map_info information about the grid
 * @return std::optional<geometry_msgs::msg::Point> world point index or nullopt if failed
 */
std::optional<geometry_msgs::msg::Point> grid_index_to_world_pt(
  const grid_index_t & grid_index,
  const nav_msgs::msg::MapMetaData & map_info);

}  // namespace wombat_core
