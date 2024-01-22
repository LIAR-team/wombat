// Copyright 2024 Soragna Alberto.
// All Rights Reserved.

#pragma once

#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Differential kinematic model
 */
class DiffKinematicModel
{
public:
  DiffKinematicModel() = default;

  void update(
    const geometry_msgs::msg::Twist & vel_msg,
    const rclcpp::Duration & delta_time);

  geometry_msgs::msg::Pose get_pose();

  void reset_pose(const geometry_msgs::msg::Pose & new_pose);

private:
  geometry_msgs::msg::Pose m_current_pose;
};
