// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <algorithm>
#include <climits>
#include <cmath>

#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/raytrace.hpp"
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
  const grid_coord_t & from_grid,
  const grid_coord_t & to_grid,
  const MapMetaDataAdapter & map_info,
  const std::function<bool(grid_index_t)> & predicate,
  const double_range_t & length_range)
{
  if (!predicate) {
    throw std::runtime_error("Invalid predicate function");
  }
  if (!grid_coord_is_valid(from_grid, map_info) || !grid_coord_is_valid(to_grid, map_info)) {
    return std::nullopt;
  }

  const int dx_full = static_cast<int>(to_grid.x() - from_grid.x());
  const int dy_full = static_cast<int>(to_grid.y() - from_grid.y());

  // we need to chose how much to scale our dominant dimension,
  // based on the maximum length of the line
  const double dist = std::hypot(dx_full, dy_full);
  if (dist < length_range.min) {
    return std::nullopt;
  }

  grid_coord_t min_from_grid;
  if (dist > 0.0) {
    // Adjust starting point and offset to start from length_range.min distance
    min_from_grid.x() = static_cast<grid_coord_t::Scalar>(from_grid.x() + dx_full / dist * length_range.min);
    min_from_grid.y() = static_cast<grid_coord_t::Scalar>(from_grid.y() + dy_full / dist * length_range.min);
  } else {
    // dist can be 0 if [from_grid.x, from_grid.y]==[to_grid.x, to_grid.y].
    // In this case only this cell should be processed.
    min_from_grid = from_grid;
  }
  const auto maybe_from_offset = wombat_core::grid_coord_to_index(min_from_grid, map_info);
  if (!maybe_from_offset) {
    throw std::runtime_error("Failed to compute from offset");
  }

  const int dx = to_grid.x() - min_from_grid.x();
  const int dy = to_grid.y() - min_from_grid.y();

  const unsigned int abs_dx = std::abs(dx);
  const unsigned int abs_dy = std::abs(dy);

  const int offset_dx = wombat_core::sign(dx);
  const int offset_dy = wombat_core::sign(dy) * map_info.grid_size.x();

  const double scale = (dist == 0.0) ? 1.0 : std::min(1.0, length_range.max / dist);

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
  const MapMetaDataAdapter & map_info,
  double angle)
{
  if (start.x() >= map_info.grid_size.x() || start.y() >= map_info.grid_size.y()) {
    return std::nullopt;
  }

  // Given a parametrized representation of the line as
  // x(t) = start.x() + t * cos(theta)
  // y(t) = start.y() + t * sin(theta)
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
    const double t_top = (map_info.grid_size.y() - static_cast<double>(start.y())) / dir_y;
    if (t_top >= 0) {
      t = std::min(t, t_top);
    }
  } else if (dir_y < 0) {
    const double t_bottom = -static_cast<double>(start.y()) / dir_y;
    if (t_bottom >= 0) {
      t = std::min(t, t_bottom);
    }
  }
  if (dir_x > 0) {
    const double t_right = (map_info.grid_size.x() - static_cast<double>(start.x())) / dir_x;
    if (t_right >= 0) {
      t = std::min(t, t_right);
    }
  } else if (dir_x < 0) {
    const double t_left = -static_cast<double>(start.x()) / dir_x;
    if (t_left >= 0) {
      t = std::min(t, t_left);
    }
  }

  // This shouldn't happen
  if (t == std::numeric_limits<double>::infinity()) {
    return std::nullopt;
  }

  const double intersection_x = start.x() + t * dir_x;
  const double intersection_y = start.y() + t * dir_y;

  // Ensure correctness of intersection coordinate
  // is this necessary?
  grid_coord_t intersection_coord {
    std::clamp(static_cast<grid_coord_t::Scalar>(intersection_x), 0, map_info.grid_size.x()),
    std::clamp(static_cast<grid_coord_t::Scalar>(intersection_y), 0, map_info.grid_size.y())
  };
  return intersection_coord;
}

}  // namespace wombat_core
