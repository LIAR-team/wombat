// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <climits>
#include <cmath>

#include "wombat_core/math/grid/coordinates.hpp"
#include "wombat_core/math/grid/raytrace.hpp"
#include "wombat_core/math/utils.hpp"

namespace wombat_core
{

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
  const grid_coord_t & start,
  const grid_coord_t & to_grid,
  const nav_msgs::msg::MapMetaData & map_info,
  const std::function<bool(grid_index_t)> & predicate,
  double min_length,
  double max_length)
{
  if (!predicate) {
    throw std::runtime_error("Invalid predicate function");
  }

  const int dx_full = static_cast<int>(to_grid.x - start.x);
  const int dy_full = static_cast<int>(to_grid.y - start.y);

  // we need to chose how much to scale our dominant dimension,
  // based on the maximum length of the line
  const double dist = std::hypot(dx_full, dy_full);
  if (dist < min_length) {
    return std::nullopt;
  }

  grid_coord_t min_from_grid;
  if (dist > 0.0) {
    // Adjust starting point and offset to start from min_length distance
    min_from_grid.x = static_cast<unsigned int>(start.x + dx_full / dist * min_length);
    min_from_grid.y = static_cast<unsigned int>(start.y + dy_full / dist * min_length);
  } else {
    // dist can be 0 if [start.x, start.y]==[to_grid.x, to_grid.y].
    // In this case only this cell should be processed.
    min_from_grid.x = start.x;
    min_from_grid.y = start.y;
  }
  const auto maybe_from_offset = wombat_core::grid_coord_to_index(min_from_grid, map_info);
  if (!maybe_from_offset) {
    throw std::runtime_error("Failed to compute from offset");
  }

  const int dx = static_cast<int>(to_grid.x - min_from_grid.x);
  const int dy = static_cast<int>(to_grid.y - min_from_grid.y);

  const unsigned int abs_dx = std::abs(dx);
  const unsigned int abs_dy = std::abs(dy);

  const int offset_dx = wombat_core::sign(dx);
  const int offset_dy = wombat_core::sign(dy) * static_cast<int>(map_info.width);

  const double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

  if (abs_dx >= abs_dy) {
    // if x is dominant
    const int error_y = static_cast<int>(abs_dx) / 2;
    return bresenham2D(
      predicate, abs_dx, abs_dy, error_y, offset_dx, offset_dy, *maybe_from_offset,
      static_cast<unsigned int>(scale * abs_dx));
  } else {
    // otherwise y is dominant
    const int error_x = static_cast<int>(abs_dy) / 2;
    return bresenham2D(
      predicate, abs_dy, abs_dx, error_x, offset_dy, offset_dx, *maybe_from_offset,
      static_cast<unsigned int>(scale * abs_dy));
  }
}

std::optional<grid_coord_t> project_to_grid_boundary(
  const grid_coord_t & start,
  const nav_msgs::msg::MapMetaData & map_info,
  double angle)
{
  if (start.x >= map_info.width || start.y >= map_info.height) {
    return std::nullopt;
  }

  // Given a parametrized representation of the line as
  // x(t) = start.x + t * cos(theta)
  // y(t) = start.y + t * sin(theta)
  // We want to find the value t for which the line intersects the boundary
  // this means that either x(t) = 0 or x(t) = width or y(t) = 0 or y(t) = height.
  // We select the smallest of these t values to denote the closest intersection point.
  // After finding t we can just plug it in the parametrized equation and compute
  // the intersection coordinate.

  const double dir_x = std::cos(angle);
  const double dir_y = std::sin(angle);

  // Find the minimum positive t
  double t = std::numeric_limits<double>::infinity();
  if (dir_y > 0) {
    const double t_top = (map_info.height - static_cast<double>(start.y)) / dir_y;
    if (t_top >= 0) {
      t = std::min(t, t_top);
    }
  } else if (dir_y < 0) {
    const double t_bottom = -static_cast<double>(start.y) / dir_y;
    if (t_bottom >= 0) {
      t = std::min(t, t_bottom);
    }
  }
  if (dir_x > 0) {
    const double t_right = (map_info.width - static_cast<double>(start.x)) / dir_x;
    if (t_right >= 0) {
      t = std::min(t, t_right);
    }
  } else if (dir_x < 0) {
    const double t_left = -static_cast<double>(start.x) / dir_x;
    if (t_left >= 0) {
      t = std::min(t, t_left);
    }
  }

  // This shouldn't happen
  if (t == std::numeric_limits<double>::infinity()) {
    return std::nullopt;
  }

  const double intersection_x = start.x + t * dir_x;
  const double intersection_y = start.y + t * dir_y;

  // Ensure correctness of intersection coordinate
  // is this necessary?
  grid_coord_t intersection_coord;
  intersection_coord.x = std::min(map_info.width, static_cast<uint32_t>(std::max(0.0, intersection_x)));
  intersection_coord.y = std::min(map_info.height, static_cast<uint32_t>(std::max(0.0, intersection_y)));

  return intersection_coord;
}

}  // namespace wombat_core
