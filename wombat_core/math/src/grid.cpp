// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <climits>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "wombat_core/math/grid.hpp"
#include "wombat_core/math/utils.hpp"

namespace wombat_core
{

std::optional<grid_index_t> grid_coord_to_index(
  const grid_coord_t & grid_coord,
  const nav_msgs::msg::MapMetaData & map_info)
{
  if (grid_coord.x > map_info.width || grid_coord.y > map_info.height) {
    return std::nullopt;
  }
  return grid_coord.y * map_info.width + grid_coord.x;
}

std::optional<grid_coord_t> world_pt_to_grid_coord(
  const geometry_msgs::msg::Point & world_pt,
  const nav_msgs::msg::MapMetaData & map_info)
{
  const auto & map_origin_x = map_info.origin.position.x;
  const auto & map_origin_y = map_info.origin.position.y;

  if (world_pt.x < map_origin_x || world_pt.y < map_origin_y) {
    return std::nullopt;
  }
  const float map_resolution = map_info.resolution;
  grid_coord_t grid_coord;
  grid_coord.x =
    static_cast<unsigned int>((world_pt.x - map_origin_x) / map_resolution);
  grid_coord.y =
    static_cast<unsigned int>((world_pt.y - map_origin_y) / map_resolution);

  return grid_coord;
}

std::optional<grid_index_t> world_pt_to_grid_index(
  const geometry_msgs::msg::Point & world_pt,
  const nav_msgs::msg::MapMetaData & map_info)
{
  const auto maybe_map_coord = world_pt_to_grid_coord(world_pt, map_info);
  if (!maybe_map_coord) {
    return std::nullopt;
  }
  return grid_coord_to_index(*maybe_map_coord, map_info);
}

// This is a private function so don't bother cleaning up the parameters.
// Fix this before making it public.
// NOLINTBEGIN(bugprone-easily-swappable-parameters)
/**
  * @brief  A 2D implementation of Bresenham's raytracing algorithm...
  * applies an action at each step
  */
static std::optional<grid_index_t> bresenham2D(
  const std::function<bool(grid_index_t)> & predicate,
  unsigned int abs_da, unsigned int abs_db, int error_b,
  int offset_a, int offset_b,
  unsigned int offset,
  unsigned int max_length)
// NOLINTEND(bugprone-easily-swappable-parameters)
{
  unsigned int end = std::min(max_length, abs_da);
  bool should_stop = false;
  for (unsigned int i = 0; i < end; ++i) {
    should_stop = predicate(offset);
    if (should_stop) {
      return offset;
    }
    offset += offset_a;
    error_b += static_cast<int>(abs_db);
    if (static_cast<unsigned int>(error_b) >= abs_da) {
      offset += offset_b;
      error_b -= static_cast<int>(abs_da);
    }
  }
  should_stop = predicate(offset);
  if (should_stop) {
    return offset;
  }
  return std::nullopt;
}

std::optional<grid_index_t> find_if_raytrace(
  const grid_coord_t & from_grid,
  const grid_coord_t & to_grid,
  const nav_msgs::msg::MapMetaData & map_info,
  const std::function<bool(grid_index_t)> & predicate)
{
  static constexpr unsigned int MAX_LENGTH = UINT_MAX;
  static constexpr unsigned int MIN_LENGTH = 0;

  std::optional<grid_index_t> output_grid_pt;

  if (!predicate) {
    throw std::runtime_error("Invalid predicate function");
  }

  int dx_full = static_cast<int>(to_grid.x - from_grid.x);
  int dy_full = static_cast<int>(to_grid.y - from_grid.y);

  // we need to chose how much to scale our dominant dimension,
  // based on the maximum length of the line
  double dist = std::hypot(dx_full, dy_full);
  if (dist < MIN_LENGTH) {
    return output_grid_pt;
  }

  grid_coord_t min_from_grid;
  if (dist > 0.0) {
    // Adjust starting point and offset to start from MIN_LENGTH distance
    min_from_grid.x = static_cast<unsigned int>(from_grid.x + dx_full / dist * MIN_LENGTH);
    min_from_grid.y = static_cast<unsigned int>(from_grid.y + dy_full / dist * MIN_LENGTH);
  } else {
    // dist can be 0 if [from_grid.x, from_grid.y]==[to_grid.x, to_grid.y].
    // In this case only this cell should be processed.
    min_from_grid.x = from_grid.x;
    min_from_grid.y = from_grid.y;
  }
  auto maybe_from_offset = grid_coord_to_index(min_from_grid, map_info);
  if (!maybe_from_offset) {
    throw std::runtime_error("Failed to compute from offset");
  }

  int dx = static_cast<int>(to_grid.x - min_from_grid.x);
  int dy = static_cast<int>(to_grid.y - min_from_grid.y);

  unsigned int abs_dx = std::abs(dx);
  unsigned int abs_dy = std::abs(dy);

  int offset_dx = wombat_core::sign(dx);
  int offset_dy = wombat_core::sign(dy) * static_cast<int>(map_info.width);

  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, MAX_LENGTH / dist);

  if (abs_dx >= abs_dy) {
    // if x is dominant
    int error_y = static_cast<int>(abs_dx) / 2;
    output_grid_pt = bresenham2D(
      predicate, abs_dx, abs_dy, error_y, offset_dx, offset_dy, *maybe_from_offset,
      static_cast<unsigned int>(scale * abs_dx));
  } else {
    // otherwise y is dominant
    int error_x = static_cast<int>(abs_dy) / 2;
    output_grid_pt = bresenham2D(
      predicate, abs_dy, abs_dx, error_x, offset_dy, offset_dx, *maybe_from_offset,
      static_cast<unsigned int>(scale * abs_dy));
  }

  return output_grid_pt;
}

}  // namespace wombat_core
