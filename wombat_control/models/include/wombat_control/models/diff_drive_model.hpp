// Copyright 2021-2022 Azzollini Ilario, Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/duration.hpp"

namespace wombat_control
{

/**
 * @brief Integrate a velocity command over the specified delta time
 * and computes an update robot pose
 * @param robot_pose current robot pose
 * @param vel_msg velocity command
 * @param delta_time integration time
 * @return geometry_msgs::msg::Pose updated pose
 */
geometry_msgs::msg::Pose diff_drive_model_integration(
  const geometry_msgs::msg::Pose & robot_pose,
  const geometry_msgs::msg::Twist & vel_msg,
  const rclcpp::Duration & delta_time);

}  // namespace wombat_control
