// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "wombat_core/grid/coordinates.hpp"

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

std::optional<grid_coord_t> grid_index_to_coord(
  const grid_index_t & grid_index,
  const nav_msgs::msg::MapMetaData & map_info)
{
  auto grid_flat_container_size =
    static_cast<grid_index_t>(map_info.width) * static_cast<grid_index_t>(map_info.height);
  if (grid_index >= grid_flat_container_size) {
    return std::nullopt;
  }
  grid_coord_t grid_coord;
  grid_coord.y = grid_index / map_info.width;
  grid_coord.x = grid_index - static_cast<grid_index_t>(grid_coord.y * map_info.width);

  return grid_coord;
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

  if (grid_coord.x >= map_info.width || grid_coord.y >= map_info.height) {
    return std::nullopt;
  }

  return grid_coord;
}

std::optional<geometry_msgs::msg::Point> grid_coord_to_world_pt(
  const grid_coord_t & grid_coord,
  const nav_msgs::msg::MapMetaData & map_info)
{
  if (grid_coord.x > map_info.width || grid_coord.y > map_info.height) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point world_pt;
  world_pt.x = map_info.origin.position.x + (grid_coord.x + 0.5) * map_info.resolution;
  world_pt.y = map_info.origin.position.y + (grid_coord.y + 0.5) * map_info.resolution;

  return world_pt;
}

std::optional<grid_index_t> world_pt_to_grid_index(
  const geometry_msgs::msg::Point & world_pt,
  const nav_msgs::msg::MapMetaData & map_info)
{
  const auto maybe_grid_coord = world_pt_to_grid_coord(world_pt, map_info);
  if (!maybe_grid_coord) {
    return std::nullopt;
  }
  return grid_coord_to_index(*maybe_grid_coord, map_info);
}

std::optional<geometry_msgs::msg::Point> grid_index_to_world_pt(
  const grid_index_t & grid_index,
  const nav_msgs::msg::MapMetaData & map_info)
{
  const auto maybe_grid_coord = grid_index_to_coord(grid_index, map_info);
  if (!maybe_grid_coord) {
    return std::nullopt;
  }
  return grid_coord_to_world_pt(*maybe_grid_coord, map_info);
}

}  // namespace wombat_core
