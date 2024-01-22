// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <cmath>
#include <memory>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "kennel/diff-kinematic-model.hpp"
#include "wombat_core/math/angles.hpp"

void DiffKinematicModel::update(
  const geometry_msgs::msg::Twist & vel_msg,
  const rclcpp::Duration & delta_time)
{
  double theta = tf2::getYaw(m_current_pose.orientation);

  m_current_pose.position.x += vel_msg.linear.x * std::cos(theta) * delta_time.seconds();
  m_current_pose.position.y += vel_msg.linear.x * std::sin(theta) * delta_time.seconds();
  m_current_pose.position.z = 0.0;

  theta += vel_msg.angular.z * delta_time.seconds();

  m_current_pose.orientation = wombat_core::quaternion_from_rpy(0.0, 0.0, theta);
}

geometry_msgs::msg::Pose DiffKinematicModel::get_pose()
{
  return m_current_pose;
}

void DiffKinematicModel::reset_pose(const geometry_msgs::msg::Pose & new_pose)
{
  m_current_pose = new_pose;
}
