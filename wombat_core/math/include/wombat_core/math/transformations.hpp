// Copyright 2024 Soragna Alberto.

#pragma once

#include <optional>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace wombat_core
{

/**
 * @brief Given two transformations with the same child frame, such as
 * a_T_x (a transformation from frame "a" to frame "x") and b_T_x
 * (a transformation from frame "b" to frame "x"), this function computes
 * a new transformation a_T_b that allows a tree-like composition of the transformations
 * retaining all the information (i.e. a_T_x = a_T_b * b_T_x)
 * @param a_T_x_msg transformation from a to x
 * @param b_T_x_msg transformation from b to x
 * @return a_T_B transformation from a to b
 */
std::optional<geometry_msgs::msg::TransformStamped>
compose_tfs(
  const geometry_msgs::msg::TransformStamped & a_T_x_msg,
  const geometry_msgs::msg::TransformStamped & b_T_x_msg);

/**
 * @brief Converts a pose message data-structure into a transformation message
 * data structure
 * @param pose the pose to convert
 * @return geometry_msgs::msg::Transform the corresponding transformation
 */
geometry_msgs::msg::Transform pose_to_transform(
  const geometry_msgs::msg::Pose & pose);

/**
 * @brief Converts a transformation message data-structure into a pose message
 * data-structure.
 * @param transform the transform to convert
 * @return geometry_msgs::msg::Pose the corresponding pose
 */
geometry_msgs::msg::Pose transform_to_pose(
  const geometry_msgs::msg::Transform & transform);

}  // namespace wombat_core
