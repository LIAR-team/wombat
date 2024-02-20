// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <cmath>
#include <memory>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kennel/common/sensors/range_projection.hpp"
#include "wombat_core/math/geometry_point.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/raytrace.hpp"

namespace kennel
{

std::vector<float> compute_laser_ranges(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & laser_pose,
  size_t num_bins,
  const std::pair<float, float> & angle_range,
  const std::pair<float, float> & distance_range)
{
  // Validate input data
  if (angle_range.first > angle_range.second) {
    return std::vector<float>();
  }
  if (distance_range.first > distance_range.second) {
    return std::vector<float>();
  }
  if (num_bins == 0) {
    return std::vector<float>();
  }

  const auto maybe_laser_coord = wombat_core::world_pt_to_grid_coord(
    laser_pose.position,
    map.info);
  if (!maybe_laser_coord) {
    return std::vector<float>();
  }
  const auto maybe_laser_index = wombat_core::grid_coord_to_index(
    *maybe_laser_coord,
    map.info);
  if (!maybe_laser_index) {
    return std::vector<float>();
  }

  const double angle_increment = (angle_range.second - angle_range.first) / static_cast<double>(num_bins);
  const double laser_yaw = tf2::getYaw(laser_pose.orientation);

  std::vector<float> ranges(num_bins);
  for (size_t i = 0; i < ranges.size(); i++) {
    const double this_angle = laser_yaw + angle_range.first + angle_increment * static_cast<double>(i);
    // Compute laser end point
    // We can't just project the distance along the angle because the grid coordinate
    // is an unsigned number and it makes sense only within the grid.
    // We need to compute the intersection between this line and the grid boundaries
    auto maybe_boundary_coord = wombat_core::project_to_grid_boundary(
      *maybe_laser_coord,
      map.info,
      this_angle);
    if (!maybe_boundary_coord) {
      return std::vector<float>();
    }

    // Raytrace between laser coord and boundary coord
    const auto maybe_obstacle_index = wombat_core::find_if_raytrace(
      *maybe_laser_coord,
      *maybe_boundary_coord,
      map.info,
      [&map](wombat_core::grid_index_t index) {
        const bool is_obstacle = map.data[index] > 0;
        return is_obstacle;
      });

    // Select a range end coordinate: either the end point or the obstacle (if any)
    wombat_core::grid_coord_t range_end_coord = *maybe_boundary_coord;
    if (maybe_obstacle_index) {
      auto maybe_obstacle_coord = wombat_core::grid_index_to_coord(*maybe_obstacle_index, map.info);
      if (!maybe_obstacle_coord) {
        return std::vector<float>();
      }
      range_end_coord = *maybe_obstacle_coord;
    }

    // Convert end coordinate to world point
    auto maybe_end_point = wombat_core::grid_coord_to_world_pt(range_end_coord, map.info);
    if (!maybe_end_point) {
      return std::vector<float>();
    }

    ranges[i] = static_cast<float>(
      wombat_core::points_distance_2d(
        laser_pose.position, *maybe_end_point));
  }

  return ranges;
}

}  // namespace kennel
