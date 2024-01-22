// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "wombat_core/math/grid.hpp"

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

  if (world_pt.x < map_origin_x || world_pt.y < map_origin_y)
  {
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

std::optional<grid_index_t> find_grid_coord_on_line(
  const geometry_msgs::msg::Point & from_world,
  const geometry_msgs::msg::Point & to_world,
  const nav_msgs::msg::MapMetaData & map_info,
  std::function<bool(grid_index_t)> eval_func)
{
  // TODO: use a better algorithm (e.g. Bresenham’s)
  // This is not precise and very inefficient.

  if (!eval_func) {
    throw std::runtime_error("No eval func");
  }

  const double dx = to_world.x - from_world.x;
  const double dy = to_world.y - from_world.y;
  const double dir_len_squared = dx*dx + dy*dy;
  if (dir_len_squared == 0) {
    return std::nullopt;
  }
  const double dir_len = std::sqrt(dir_len_squared);
  const double step_x = dx / dir_len * map_info.resolution * 0.9;
  const double step_y = dy / dir_len * map_info.resolution * 0.9;
  geometry_msgs::msg::Point current_world_point = from_world;
  while (true) {
    const double dist_from_start_squared = (to_world.x - current_world_point.x) * (to_world.x - current_world_point.x) +
      (to_world.y - current_world_point.y) * (to_world.y - current_world_point.y); 
    if (dist_from_start_squared > dir_len_squared) {
      break;
    }
    auto maybe_map_index = world_pt_to_grid_index(current_world_point, map_info);
    if (!maybe_map_index) {
      throw std::runtime_error("Bad map index");
    }
    if (eval_func(*maybe_map_index)) {
      return *maybe_map_index;
    }
    current_world_point.x += step_x;
    current_world_point.y += step_y;
  }
  return std::nullopt;
}

}  // namespace wombat_core
