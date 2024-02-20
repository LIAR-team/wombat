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

bool grid_coord_is_valid(
  const wombat_core::grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info);

bool grid_index_is_valid(
  const wombat_core::grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info);

wombat_core::grid_coord_t enfouce_bounds_on_grid_coord(
  const wombat_core::grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info);

wombat_core::grid_size_t get_subgrid_size_from_corners(
  const wombat_core::grid_coord_t & top_left_grid_coord,
  const wombat_core::grid_coord_t & bottom_right_grid_coord);

wombat_core::grid_coord_t grid_coord_bounded_diff(
  const wombat_core::grid_coord_t & minuend,
  const wombat_core::grid_coord_t & subtrahend);

wombat_core::grid_coord_t grid_coord_bounded_sum(
  const wombat_core::grid_coord_t & minuend,
  const wombat_core::grid_coord_t & subtrahend,
  const MapMetaDataAdapter & map_info);

/*!
 * Increases the index by one to iterate through the cells of a submap.
 * Increments either to the neighboring index to the right or to
 * the start of the lower row. Returns false if end of iterations are reached.
 *
 * Note: This function does not check if submap actually fits to the map. This needs
 * to be checked before separately.
 *
 * @param[in/out] submapIndex the index in the submap that is incremented.
 * @param[out] index the index in the map that is incremented (corrected for the circular buffer).
 * @param[in] submapTopLefIndex the top left index of the submap.
 * @param[in] submapBufferSize the submap buffer size.
 * @param[in] bufferStartIndex the map buffer start index.
 * @return true if successfully incremented indices, false if end of iteration limits are reached.
 */
bool increment_index_for_submap(
  grid_coord_t & submapIndex,
  grid_coord_t & index,
  const grid_coord_t & submapTopLeftIndex,
  const MapMetaDataAdapter & submap_info);

}  // namespace wombat_core
