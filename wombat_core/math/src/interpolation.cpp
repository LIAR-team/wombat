// Copyright 2024 Soragna Alberto.

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "wombat_core/math/interpolation.hpp"

namespace wombat_core
{

geometry_msgs::msg::Pose pose_interpolation(
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & end_pose,
  double factor)
{
  geometry_msgs::msg::Pose interpolated_pose;

  // Interpolate position
  interpolated_pose.position.x = wombat_core::linear_interpolation(
    start_pose.position.x,
    end_pose.position.x,
    factor);
  interpolated_pose.position.y = wombat_core::linear_interpolation(
    start_pose.position.y,
    end_pose.position.y,
    factor);
  interpolated_pose.position.z = wombat_core::linear_interpolation(
    start_pose.position.z,
    end_pose.position.z,
    factor);

  // Interpolate orientation
  tf2::Quaternion start_quat;
  tf2::fromMsg(start_pose.orientation, start_quat);
  tf2::Quaternion end_quat;
  tf2::fromMsg(end_pose.orientation, end_quat);

  const tf2::Quaternion interpolated_quat = tf2::slerp(start_quat, end_quat, factor);
  interpolated_pose.orientation = tf2::toMsg(interpolated_quat);

  return interpolated_pose;
}

}  // namespace wombat_core
