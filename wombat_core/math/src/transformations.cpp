// Copyright 2024 Soragna Alberto.

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "wombat_core/math/transformations.hpp"

namespace wombat_core
{

std::optional<geometry_msgs::msg::TransformStamped>
compose_tfs(
  const geometry_msgs::msg::TransformStamped & a_T_x_msg,
  const geometry_msgs::msg::TransformStamped & b_T_x_msg)
{
  if (a_T_x_msg.child_frame_id != b_T_x_msg.child_frame_id) {
    return std::nullopt;
  }

  // Compute the new transformation
  tf2::Transform a_T_x;
  tf2::fromMsg(a_T_x_msg.transform, a_T_x);
  tf2::Transform b_T_x;
  tf2::fromMsg(b_T_x_msg.transform, b_T_x);
  const tf2::Transform a_T_b = a_T_x * b_T_x.inverse();

  geometry_msgs::msg::TransformStamped a_T_b_msg;
  a_T_b_msg.header = a_T_x_msg.header;
  a_T_b_msg.child_frame_id = b_T_x_msg.header.frame_id;
  tf2::toMsg(a_T_b, a_T_b_msg.transform);

  return a_T_b_msg;
}

geometry_msgs::msg::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
  transform.rotation.x = pose.orientation.x;
  transform.rotation.y = pose.orientation.y;
  transform.rotation.z = pose.orientation.z;
  transform.rotation.w = pose.orientation.w;

  return transform;
}

geometry_msgs::msg::Pose transform_to_pose(const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;

  return pose;
}


}  // namespace wombat_core
