// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/common/collisions.hpp"
#include "kennel/mobile_base/ground_truth_manager.hpp"
#include "wombat_control/models/diff_drive_model.hpp"
#include "wombat_core/math/grid/coordinates.hpp"
#include "wombat_core/math/transformations.hpp"

namespace kennel
{

GroundTruthManager::GroundTruthManager(
  rclcpp::Node * parent_node,
  std::string ground_truth_frame_id,
  const std::chrono::milliseconds & cmd_timeout,
  std::string robot_base_frame_id)
: m_clock(parent_node->get_clock()), m_logger(parent_node->get_logger()),
  m_ground_truth_frame_id(std::move(ground_truth_frame_id)), m_robot_base_frame_id(std::move(robot_base_frame_id)),
  m_cmd_timeout(cmd_timeout)
{
  m_costmap = std::make_unique<wombat_core::InflatableCostmap>();

  RCLCPP_INFO(m_logger, "Ground truth manager constructed");
}

void
GroundTruthManager::map_update(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_map = map;
  m_costmap->update_map(map);
  // We render after every map update, as we expect them to be very infrequent
  m_costmap->render_costmap();
}

std::string GroundTruthManager::get_ground_truth_frame_id() const
{
  return m_ground_truth_frame_id;
}

nav2_costmap_2d::Costmap2D * GroundTruthManager::get_costmap()
{
  return m_costmap->get_inflated_costmap();
}

geometry_msgs::msg::TransformStamped
GroundTruthManager::pose_update(const geometry_msgs::msg::TwistStamped & cmd_vel)
{
  const auto now = m_clock->now();

  // TODO: restrict scope of mutex
  std::lock_guard<std::mutex> lock(m_mutex);

  m_gt_transform.header.stamp = now;
  m_gt_transform.header.frame_id = m_map ? m_map->header.frame_id : m_ground_truth_frame_id;
  m_gt_transform.child_frame_id = m_robot_base_frame_id;

  if (!m_last_pose_update_time) {
    m_last_pose_update_time = now;
    m_gt_transform.transform = wombat_core::pose_to_transform(m_gt_pose);
    return m_gt_transform;
  }

  // Do not apply commands that are too old
  const auto dt_since_last_cmd_received = now - rclcpp::Time(cmd_vel.header.stamp);
  const bool cmd_is_valid = dt_since_last_cmd_received < m_cmd_timeout;

  if (cmd_is_valid) {
    const auto dt_since_last_pose_update = now - m_last_pose_update_time.value();
    auto new_pose = wombat_control::diff_drive_model_integration(
      m_gt_pose,
      cmd_vel.twist,
      dt_since_last_pose_update);

    // Validate the new pose before updating the ground truth member variable
    if (m_map) {
      m_gt_pose = kennel::apply_map_collisions(*m_map, m_gt_pose, new_pose);
    } else {
      m_gt_pose = new_pose;
    }
  }

  m_last_pose_update_time = now;
  m_gt_transform.transform = wombat_core::pose_to_transform(m_gt_pose);

  return m_gt_transform;
}

geometry_msgs::msg::TransformStamped GroundTruthManager::get_pose() const
{
  return m_gt_transform;
}

void GroundTruthManager::reset_pose(const geometry_msgs::msg::Pose & new_pose)
{
  m_last_pose_update_time = m_clock->now();
  m_gt_pose = new_pose;
}

}  // namespace kennel
