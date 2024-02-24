// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace kennel
{

/**
 * @brief Determines whether the robot can move from the start to the
 * end poses without colliding with obstacles.
 * The function returns the pose furthest from start that is reachable
 * without collisions.
 * The function assumes that the start and end poses are close enough
 * that the motion between them can be considered a single arc of circumference.
 * This function assumes that the start pose is collision free.
 * @note This function currently assumes a circular robot.
 * @param map map where to check for collisions
 * @param start_pose collision-free pose where the robot starts from
 * @param input_end_pose desired end pose, potentially in collision
 * @return geometry_msgs::msg::Pose new pose of the robot after
 * trying to move from start to end and stopping when "bumping" into an obstacle.
 */
geometry_msgs::msg::Pose apply_map_collisions(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & input_end_pose);

}  // namespace kennel
