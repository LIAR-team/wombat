// Copyright 2024 Soragna Alberto.

#include <cmath>
#include <memory>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kennel/common/range_projection.hpp"
#include "wombat_core/math/geometry_point.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/raytrace.hpp"

namespace kennel
{

static wombat_core::grid_coord_t bound_coord_with_projection(
  const wombat_core::grid_coord_t & grid_coord,
  double angle,
  const wombat_core::grid_coord_t & origin_coord,
  const wombat_core::MapMetaDataAdapter & map_info)
{
  if (wombat_core::grid_coord_is_valid(grid_coord, map_info)) {
    return grid_coord;
  }

  if (!wombat_core::grid_coord_is_valid(origin_coord, map_info)) {
    throw std::runtime_error("Origin coord must be valid for projection");
  }
  auto maybe_boundary_coord = wombat_core::project_to_grid_boundary(
    origin_coord,
    map_info,
    angle);
  if (!maybe_boundary_coord) {
    throw std::runtime_error("Failed to project grid coord");
  }
  return *maybe_boundary_coord;
}

std::vector<wombat_core::grid_coord_t> compute_laser_projections(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & laser_pose,
  size_t num_bins,
  const std::pair<float, float> & angle_range,
  const std::pair<float, float> & distance_range)
{
  // Validate input data
  if (angle_range.first > angle_range.second) {
    return std::vector<wombat_core::grid_coord_t>();
  }
  if (distance_range.first > distance_range.second) {
    return std::vector<wombat_core::grid_coord_t>();
  }
  if (num_bins == 0) {
    return std::vector<wombat_core::grid_coord_t>();
  }

  auto map_info = wombat_core::MapMetaDataAdapter(map.info);

  const auto maybe_laser_coord = wombat_core::world_pt_to_grid_coord(
    laser_pose.position,
    map_info);
  if (!maybe_laser_coord) {
    return std::vector<wombat_core::grid_coord_t>();
  }
  const auto maybe_laser_index = wombat_core::grid_coord_to_index(
    *maybe_laser_coord,
    map_info);
  if (!maybe_laser_index) {
    return std::vector<wombat_core::grid_coord_t>();
  }

  const double angle_increment = (angle_range.second - angle_range.first) / static_cast<double>(num_bins);
  const double laser_yaw = tf2::getYaw(laser_pose.orientation);
  const auto grid_range = wombat_core::double_range_t{
    distance_range.first / map_info.resolution,
    distance_range.second / map_info.resolution};

  std::vector<wombat_core::grid_coord_t> end_coords(num_bins);
  for (size_t i = 0; i < end_coords.size(); i++) {
    const double this_angle = laser_yaw + angle_range.first + angle_increment * static_cast<double>(i);
    // Compute laser end point
    // We can't just project the distance along the angle because the grid coordinate
    // is an unsigned number and it makes sense only within the grid.
    // We need to compute the intersection between this line and the grid boundaries

    wombat_core::grid_coord_t projected_coord = {
      maybe_laser_coord->x() + std::cos(this_angle) * grid_range.max,
      maybe_laser_coord->y() + std::sin(this_angle) * grid_range.max,
    };
    projected_coord = bound_coord_with_projection(projected_coord, this_angle, *maybe_laser_coord, map_info);

    // Raytrace between laser coord and boundary coord
    auto last_touched_index = *maybe_laser_index;
    wombat_core::find_if_raytrace(
      *maybe_laser_coord,
      projected_coord,
      map_info,
      [&map, &last_touched_index](wombat_core::grid_index_t index) {
        const bool is_obstacle = map.data[index] > 0;
        last_touched_index = index;
        return is_obstacle;
      },
      grid_range);

    // The last touched index represents the last visitable cell, being it an obstacle,
    // the end of the grid or the end of the range.
    const auto maybe_last_touched_coord = wombat_core::grid_index_to_coord(last_touched_index, map_info);
    if (!maybe_last_touched_coord) {
      throw std::runtime_error("Failed to compute last touched grid coordinate");
    }
    end_coords[i] = *maybe_last_touched_coord;
  }

  return end_coords;
}

}  // namespace kennel
