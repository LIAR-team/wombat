// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <cmath>

#include "kennel/common/collisions.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/raytrace.hpp"
#include "wombat_core/math/interpolation.hpp"

namespace kennel
{

geometry_msgs::msg::Pose apply_map_collisions(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & start_pose,
  geometry_msgs::msg::Pose end_pose)
{
  // If the map is empty we just forward the updated pose
  if (map.data.empty()) {
    return end_pose;
  }

  // No translation, there may be rotation.
  // Nothing to check under the assumption that the robot is circular
  static constexpr double EPSILON = 1e-6;
  const double start_end_dx = end_pose.position.x - start_pose.position.x;
  const double start_end_dy = end_pose.position.y - start_pose.position.y;
  if (std::abs(start_end_dx) < EPSILON && std::abs(start_end_dy) < EPSILON) {
    return end_pose;
  }

  const auto start_pose_coord = wombat_core::world_pt_to_grid_coord(start_pose.position, map.info);
  if (!start_pose_coord) {
    throw std::runtime_error("Failed to compute start pose grid coordinate");
  }
  const auto start_pose_idx = wombat_core::grid_coord_to_index(*start_pose_coord, map.info);
  if (!start_pose_idx) {
    throw std::runtime_error("Failed to compute start pose index");
  }
  if (map.data[*start_pose_idx] != 0) {
    throw std::runtime_error("Start pose is not a free cell");
  }

  auto end_pose_coord = wombat_core::world_pt_to_grid_coord(end_pose.position, map.info);
  if (!end_pose_coord) {
    // The end pose may not have a valid corresponding grid coordinate as it was
    // computed by applying a world motion to the start pose.
    // We need to clamp it to ensure that it's valid before proceeding.
    const double motion_direction = std::atan2(start_end_dy, start_end_dx);
    end_pose_coord = wombat_core::project_to_grid_boundary(
      *start_pose_coord,
      map.info,
      motion_direction);
    if (!end_pose_coord) {
      throw std::runtime_error("Failed to project start pose to the grid boundary");
    }
    auto maybe_corrected_end_pt = wombat_core::grid_coord_to_world_pt(*end_pose_coord, map.info);
    if (!maybe_corrected_end_pt) {
      throw std::runtime_error("Failed to compute corrected end point");
    }
    end_pose.position = *maybe_corrected_end_pt;
  }

  auto last_free_index = wombat_core::grid_coord_to_index(*start_pose_coord, map.info);
  const auto maybe_obstacle_index = wombat_core::find_if_raytrace(
    *start_pose_coord,
    *end_pose_coord,
    map.info,
    [&map, &last_free_index](wombat_core::grid_index_t index) {
      bool is_obstacle = map.data[index] > 0;
      if (!is_obstacle) {
        last_free_index = index;
      }
      return is_obstacle;
    });

  // If we didn't find any obstacle, just return the end pose
  if (!maybe_obstacle_index) {
    return end_pose;
  }

  // We found an obstacle between the start and the end poses.
  // Select the last free cell as the new pose
  // to simulate that the robot is now in contact with the obstacle.

  const auto maybe_last_free_coord = wombat_core::grid_index_to_coord(*last_free_index, map.info);
  if (!maybe_last_free_coord) {
    throw std::runtime_error("Failed to compute last free grid coordinate");
  }

  const int fs_dx = static_cast<int>(maybe_last_free_coord->x) - static_cast<int>(start_pose_coord->x);
  const int fs_dy = static_cast<int>(maybe_last_free_coord->y) - static_cast<int>(start_pose_coord->y);

  // We couldn't move even a single cell due to the obstacle.
  // We just assume that we didn't move at all.
  if (fs_dx == 0 && fs_dy == 0) {
    return start_pose;
  }

  // We can't directly select the position corresponding to the last free cell, because,
  // due to the discrete values of the grid, it can cause instability in the robot pose.
  // For example the position could snap back and forth between two neighbor grid cell centers.
  // At the very least, we should ensure that we never move backward wrt the old pose.
  // The current implementation uses an interpolation to translate the grid motion into
  // world motion.
  // Note: we use a single scaling factor rather than computing x, y and xy independently.

  const double fs_dist =
    std::sqrt(
    static_cast<double>(fs_dx) * static_cast<double>(fs_dx) + static_cast<double>(fs_dy) * static_cast<double>(fs_dy));
  const int es_dx = static_cast<int>(end_pose_coord->x) - static_cast<int>(start_pose_coord->x);
  const int es_dy = static_cast<int>(end_pose_coord->y) - static_cast<int>(start_pose_coord->y);
  const double es_dist =
    std::sqrt(
    static_cast<double>(es_dx) * static_cast<double>(es_dx) + static_cast<double>(es_dy) * static_cast<double>(es_dy));
  const double scaling_factor = fs_dist / es_dist;

  const auto interpolated_pose = wombat_core::pose_interpolation(
    start_pose,
    end_pose,
    scaling_factor);
  return interpolated_pose;
}

}  // namespace kennel
