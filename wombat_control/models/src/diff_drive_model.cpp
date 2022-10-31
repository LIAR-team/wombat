// Copyright 2021-2022 Azzollini Ilario, Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <tf2/utils.h>

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "wombat_control/models/diff_drive_model.hpp"

DiffDriveModel::DiffDriveModel(const geometry_msgs::msg::Pose & initial_pose)
: m_current_pose(initial_pose)
{}

geometry_msgs::msg::Pose DiffDriveModel::integration(
  const geometry_msgs::msg::Twist & vel_commands,
  double dt)
{
  double theta = tf2::getYaw(m_current_pose.orientation);

  // Integration
  m_current_pose.position.x += dt * (vel_commands.linear.x * std::cos(theta));
  m_current_pose.position.y += dt * (vel_commands.linear.x * std::sin(theta));

  theta += dt * (vel_commands.angular.z);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  m_current_pose.orientation = tf2::toMsg(q);

  return m_current_pose;
}
