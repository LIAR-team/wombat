// Copyright 2021-2022 Azzollini Ilario, Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DiffDriveModel
{
public:
  explicit DiffDriveModel(const geometry_msgs::msg::Pose & initial_pose);

  // v [m/s], omega [rad/s], dt [s]
  geometry_msgs::msg::Pose integration(
    const geometry_msgs::msg::Twist & vel_commands,
    double dt);

private:
  geometry_msgs::msg::Pose m_current_pose;
};
