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
 * @brief Check if a grid coordinate is valid for the given grid,
 * i.e. if it's not out of bounds.
 * @param grid_coord the coordinate
 * @param map_info the grid
 * @return true if the coordinate is valid
 */
bool grid_coord_is_valid(
  const wombat_core::grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Check if a grid index is valid for the given grid,
 * i.e. if it's not out of bounds.
 * @param grid_coord the index
 * @param map_info the grid
 * @return true if the index is valid
 */
bool grid_index_is_valid(
  const wombat_core::grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Get the linear size of the grid, i.e. the total
 * number of grid cells in it.
 * @param map_info the grid
 * @return the size
 */
grid_index_t get_grid_linear_size(const MapMetaDataAdapter & map_info);

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
  const MapMetaDataAdapter & map_info);

/**
 * @brief Converts grid vector index (used to reference values when a grid
 * is represented as a flat container) into a grid coordinate.
 * @param grid_index the index to convert
 * @param map_info information about the grid
 * @return std::optional<grid_coord_t> grid coordinate or nullopt if failed
 */
std::optional<grid_coord_t> grid_index_to_coord(
  const grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Converts a world point into its corresponding grid
 * coordinate
 * @param world_pt the world point to convert
 * @param map_info information about the grid
 * @return std::optional<grid_coord_t> grid coord or nullopt if failed
 */
std::optional<grid_coord_t> world_pt_to_grid_coord(
  const geometry_msgs::msg::Point & world_pt,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Converts a world point into its corresponding grid
 * coordinate. If the world point is outside the grid, will return the closest
 * point on the grid's border.
 * @param world_pt the world point to convert
 * @param map_info information about the grid
 * @return grid coord
 */
grid_coord_t world_pt_to_grid_coord_enforce_bounds(
  const geometry_msgs::msg::Point & world_pt,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Converts a grid coordinate into the corresponding world point.
 * @param grid_coord_t the grid coordinate to convert
 * @param map_info information about the grid
 * @return world point or nullopt if failed
 */
std::optional<geometry_msgs::msg::Point> grid_coord_to_world_pt(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Converts a world point into the index used to access
 * the corresponding grid coordinate
 * @param world_pt the world point to convert
 * @param map_info information about the grid
 * @return std::optional<grid_index_t> grid index or nullopt if failed
 */
std::optional<grid_index_t> world_pt_to_grid_index(
  const geometry_msgs::msg::Point & world_pt,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Converts a grid index into the corresponding world point
 * @param grid_index the grid index to convert
 * @param map_info information about the grid
 * @return std::optional<geometry_msgs::msg::Point> world point index or nullopt if failed
 */
std::optional<geometry_msgs::msg::Point> grid_index_to_world_pt(
  const grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Compute a valid grid coord as the closest coord
 * to the provided, and potentially invalid, input grid coordinate
 * @param grid_coord the grid coordinate to remap into the grid
 * @param map_info the grid
 * @return wombat_core::grid_coord_t a valid grid coordinate
 */
wombat_core::grid_coord_t enfouce_bounds_on_grid_coord(
  const wombat_core::grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Compute the size of a grid box given its min and max corners
 * @param min_corner the corner with min x and min y
 * @param max_corner the corner with max x and max y
 * @return wombat_core::grid_size_t the 2D size of the box
 */
wombat_core::grid_size_t get_grid_size_from_corners(
  const wombat_core::grid_coord_t & min_corner,
  const wombat_core::grid_coord_t & max_corner);

/**
 * @brief Increases the coord by one to iterate through the cells of a grid.
 * Increments either to the neighboring coord to the right or to
 * the start of the lower row. Returns false if end of iterations are reached.
 *
 * @param[in/out] grid_coord the coord in the submap that is incremented.
 * @param[in] map_info information about the grid
 * @return true if successfully incremented the coord, false if end of grid is reached.
 */
bool increment_grid_coord(
  grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info);

/**
 * @brief Increases the coord by one to iterate through the cells of a submap.
 * Increments either to the neighboring coord to the right or to
 * the start of the lower row. Returns false if end of iterations are reached.
 *
 * Note: This function does not check if submap actually fits to the map. This needs
 * to be checked before separately.
 *
 * @param[in/out] submap_coord the coord in the submap that is incremented.
 * @param[in] submap_min_coord the min coord of the submap.
 * @param[in] submap_info information about the submap
 * @param[out] map_coord the coord in the "main" map that is incremented
 * @return true if successfully incremented indices, false if end of iteration limits are reached.
 */
bool increment_submap_coord(
  grid_coord_t & submap_coord,
  const grid_coord_t & submap_min_coord,
  const MapMetaDataAdapter & submap_info,
  grid_coord_t & map_coord);

}  // namespace wombat_core
