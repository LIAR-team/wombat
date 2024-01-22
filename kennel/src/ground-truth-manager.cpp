// Copyright 2024 Soragna Alberto.
// All Rights Reserved.

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/ground-truth-manager.hpp"
#include "kennel/diff-kinematic-model.hpp"
#include "wombat_core/math/grid.hpp"
#include "wombat_core/math/transformations.hpp"

GroundTruthManager::GroundTruthManager(
  rclcpp::Node * parent_node,
  const std::chrono::milliseconds & cmd_timeout,
  const std::string & ground_truth_frame_id,
  const std::string & robot_base_frame_id)
: m_clock(parent_node->get_clock()), m_logger(parent_node->get_logger()),
m_ground_truth_frame_id(ground_truth_frame_id), m_robot_base_frame_id(robot_base_frame_id),
m_cmd_timeout(cmd_timeout)
{
  m_kin_model = std::make_unique<DiffKinematicModel>();

  RCLCPP_INFO(m_logger, "Ground truth manager constructed");
}

geometry_msgs::msg::TransformStamped
GroundTruthManager::pose_update(
  const geometry_msgs::msg::TwistStamped & cmd_vel,
  const nav_msgs::msg::OccupancyGrid & map)
{
  const auto now = m_clock->now();
  const auto last_gt_pose = m_kin_model->get_pose();

  geometry_msgs::msg::TransformStamped gt_transform;
  gt_transform.header.stamp = now;
  gt_transform.header.frame_id = m_ground_truth_frame_id.empty() ? map.header.frame_id : m_ground_truth_frame_id;
  gt_transform.child_frame_id = m_robot_base_frame_id;

  if (!m_last_pose_update) {
    m_last_pose_update = now;
    gt_transform.transform = wombat_core::pose_to_transform(last_gt_pose);
    return gt_transform;
  }

  // Do not apply commands that are too old
  const auto dt_since_last_cmd_received = now - rclcpp::Time(cmd_vel.header.stamp);
  const bool cmd_is_valid = dt_since_last_cmd_received < m_cmd_timeout;

  if (cmd_is_valid) {
    const auto dt_since_last_pose_update = now - m_last_pose_update.value();
    m_kin_model->update(
      cmd_vel.twist,
      dt_since_last_pose_update);
  }
  const auto new_gt_pose = m_kin_model->get_pose();
  m_last_pose_update = now;

  const auto map_pose = this->apply_map_constraints(map, last_gt_pose, new_gt_pose);
  m_kin_model->reset_pose(map_pose);

  gt_transform.transform = wombat_core::pose_to_transform(map_pose);

  return gt_transform;
}

void GroundTruthManager::reset_pose(const geometry_msgs::msg::Pose & new_pose)
{
  m_last_pose_update = m_clock->now();
  m_kin_model->reset_pose(new_pose);
}

geometry_msgs::msg::Pose GroundTruthManager::apply_map_constraints(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & old_pose,
  const geometry_msgs::msg::Pose & new_pose)
{
  // If the map is empty we just forward the updated pose
  if (map.data.empty()) {
    return new_pose;
  }

  if (old_pose.position.x == new_pose.position.x && old_pose.position.y == new_pose.position.y) {
    return new_pose;
  }

  auto old_index = wombat_core::world_pt_to_grid_index(old_pose.position, map.info);
  auto new_index = wombat_core::world_pt_to_grid_index(new_pose.position, map.info);
  if (!old_index || !new_index) {
    return old_pose;
  }

  // Naive approach to verify if the robot can travel between two poses.
  auto obstacle_index = wombat_core::find_grid_coord_on_line(
    old_pose.position,
    new_pose.position,
    map.info,
    [&map](size_t index) {
      return map.data[index] > 0;
    });

  if (obstacle_index) {
    return old_pose;
  }
  return new_pose;
}
