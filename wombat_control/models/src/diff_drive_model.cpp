// Copyright 2021-2024 Azzollini Ilario, Soragna Alberto.

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "wombat_control/models/diff_drive_model.hpp"
#include "wombat_core/math/angles.hpp"

namespace wombat_control
{

geometry_msgs::msg::Pose diff_drive_model_integration(
  const geometry_msgs::msg::Pose & robot_pose,
  const geometry_msgs::msg::Twist & vel_msg,
  const rclcpp::Duration & delta_time)
{
  geometry_msgs::msg::Pose updated_pose;

  double theta = tf2::getYaw(robot_pose.orientation);

  updated_pose.position.x = robot_pose.position.x + vel_msg.linear.x * std::cos(theta) * delta_time.seconds();
  updated_pose.position.y = robot_pose.position.y + vel_msg.linear.x * std::sin(theta) * delta_time.seconds();
  updated_pose.position.z = 0.0;

  theta += vel_msg.angular.z * delta_time.seconds();

  updated_pose.orientation = wombat_core::quaternion_from_rpy(0.0, 0.0, theta);

  return updated_pose;
}

}  // namespace wombat_control
