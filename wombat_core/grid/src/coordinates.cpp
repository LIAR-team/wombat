// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "wombat_core/grid/coordinates.hpp"

namespace wombat_core
{

grid_index_t get_grid_linear_size(const MapMetaDataAdapter & map_info)
{
  return static_cast<grid_index_t>(map_info.grid_size.x()) * static_cast<grid_index_t>(map_info.grid_size.y());
}

bool grid_coord_is_valid(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  return
    grid_coord.x() >= 0 && grid_coord.x() < map_info.grid_size.x() &&
    grid_coord.y() >= 0 && grid_coord.y() < map_info.grid_size.y();
}

bool grid_index_is_valid(
  const grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info)
{
  return grid_index < get_grid_linear_size(map_info);
}

std::optional<grid_index_t> grid_coord_to_index(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  if (!grid_coord_is_valid(grid_coord, map_info)) {
    return std::nullopt;
  }
  return grid_coord.y() * map_info.grid_size.x() + grid_coord.x();
}

std::optional<grid_coord_t> grid_index_to_coord(
  const grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info)
{
  if (!grid_index_is_valid(grid_index, map_info)) {
    return std::nullopt;
  }

  grid_coord_t grid_coord;
  grid_coord.y() =
    static_cast<grid_coord_t::Scalar>(
    static_cast<grid_coord_t::Scalar>(grid_index) / map_info.grid_size.x());
  grid_coord.x() =
    static_cast<grid_coord_t::Scalar>(
    static_cast<grid_coord_t::Scalar>(grid_index) - grid_coord.y() * map_info.grid_size.x());
  return grid_coord;
}

std::optional<grid_coord_t> world_pt_to_grid_coord(
  const geometry_msgs::msg::Point & world_pt,
  const MapMetaDataAdapter & map_info)
{
  const auto & map_origin_x = map_info.origin.position.x;
  const auto & map_origin_y = map_info.origin.position.y;

  if (world_pt.x < map_origin_x || world_pt.y < map_origin_y) {
    return std::nullopt;
  }
  const float map_resolution = map_info.resolution;
  grid_coord_t grid_coord {
    static_cast<grid_coord_t::Scalar>((world_pt.x - map_origin_x) / map_resolution),
    static_cast<grid_coord_t::Scalar>((world_pt.y - map_origin_y) / map_resolution)
  };

  if (grid_coord.x() >= map_info.grid_size.x() || grid_coord.y() >= map_info.grid_size.y()) {
    return std::nullopt;
  }

  return grid_coord;
}

grid_coord_t world_pt_to_grid_coord_enforce_bounds(
  const geometry_msgs::msg::Point & world_pt,
  const MapMetaDataAdapter & map_info)
{
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  grid_coord_t grid_coord;

  const auto & origin = map_info.origin.position;

  if (world_pt.x < origin.x) {
    grid_coord.x() = 0;
  } else if (world_pt.x > map_info.resolution * static_cast<double>(map_info.grid_size.x()) + origin.x) {
    grid_coord.x() = map_info.grid_size.x() - 1;
  } else {
    grid_coord.x() = static_cast<int>((world_pt.x - origin.x) / map_info.resolution);
  }

  if (world_pt.y < origin.y) {
    grid_coord.y() = 0;
  } else if (world_pt.y > map_info.resolution * static_cast<double>(map_info.grid_size.y()) + origin.y) {
    grid_coord.y() = map_info.grid_size.y() - 1;
  } else {
    grid_coord.y() = static_cast<int>((world_pt.y - origin.y) / map_info.resolution);
  }

  return grid_coord;
}

std::optional<geometry_msgs::msg::Point> grid_coord_to_world_pt(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  if (!grid_coord_is_valid(grid_coord, map_info)) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point world_pt;
  world_pt.x = map_info.origin.position.x + (grid_coord.x() + 0.5) * map_info.resolution;
  world_pt.y = map_info.origin.position.y + (grid_coord.y() + 0.5) * map_info.resolution;

  return world_pt;
}

std::optional<grid_index_t> world_pt_to_grid_index(
  const geometry_msgs::msg::Point & world_pt,
  const MapMetaDataAdapter & map_info)
{
  const auto maybe_grid_coord = world_pt_to_grid_coord(world_pt, map_info);
  if (!maybe_grid_coord) {
    return std::nullopt;
  }
  return grid_coord_to_index(*maybe_grid_coord, map_info);
}

std::optional<geometry_msgs::msg::Point> grid_index_to_world_pt(
  const grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info)
{
  const auto maybe_grid_coord = grid_index_to_coord(grid_index, map_info);
  if (!maybe_grid_coord) {
    return std::nullopt;
  }
  return grid_coord_to_world_pt(*maybe_grid_coord, map_info);
}

static grid_coord_t::Scalar enfouce_bounds_on_grid_coord_dimension(
  grid_coord_t::Scalar coord_scalar,
  grid_size_t::Scalar size)
{
  if (coord_scalar < 0) {
    return 0;
  } else if (coord_scalar >= size) {
    return size - 1;
  }
  return coord_scalar;
}

grid_coord_t enfouce_bounds_on_grid_coord(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  return grid_coord_t{
    enfouce_bounds_on_grid_coord_dimension(grid_coord.x(), map_info.grid_size.x()),
    enfouce_bounds_on_grid_coord_dimension(grid_coord.y(), map_info.grid_size.y()),
  };
}

grid_size_t get_grid_size_from_corners(
  const grid_coord_t & min_corner,
  const grid_coord_t & max_corner)
{
  return grid_size_t{
    max_corner.x() - min_corner.x() + 1,
    max_corner.y() - min_corner.y() + 1
  };
}

bool increment_grid_coord(
  grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  if (grid_coord.x() < map_info.grid_size.x() - 1) {
    // Same row.
    grid_coord.x()++;
  } else {
    // Next row.
    grid_coord.y()++;
    grid_coord.x() = 0;
  }

  return grid_coord_is_valid(grid_coord, map_info);
}

bool increment_submap_coord(
  grid_coord_t & submap_coord,
  const grid_coord_t & submap_min_coord,
  const MapMetaDataAdapter & submap_info,
  grid_coord_t & map_coord)
{
  // Copy the data first, only copy it back if everything is within range.
  auto tmp_submap_coord = submap_coord;

  const bool valid = increment_grid_coord(tmp_submap_coord, submap_info);
  // End of submap reached.
  if (!valid) {
    return false;
  }

  // Copy data back.
  map_coord = submap_min_coord + tmp_submap_coord;
  submap_coord = tmp_submap_coord;
  return true;
}

}  // namespace wombat_core
