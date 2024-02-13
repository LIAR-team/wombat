// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <cmath>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/mobile_base/ground_truth_manager.hpp"
#include "wombat_control/models/diff_drive_model.hpp"
#include "wombat_core/math/grid/coordinates.hpp"
#include "wombat_core/math/grid/raytrace.hpp"
#include "wombat_core/math/interpolation.hpp"
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
      m_gt_pose = this->apply_map_constraints(*m_map, m_gt_pose, new_pose);
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

geometry_msgs::msg::Pose GroundTruthManager::apply_map_constraints(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & end_pose)
{
  // If the map is empty we just forward the updated pose
  if (map.data.empty()) {
    return end_pose;
  }

  if (start_pose.position.x == end_pose.position.x && start_pose.position.y == end_pose.position.y) {
    return end_pose;
  }

  auto start_pose_coord = wombat_core::world_pt_to_grid_coord(start_pose.position, map.info);
  auto end_pose_coord = wombat_core::world_pt_to_grid_coord(end_pose.position, map.info);
  if (!start_pose_coord || !end_pose_coord) {
    return start_pose;
  }

  auto last_free_index = wombat_core::grid_coord_to_index(*start_pose_coord, map.info);
  auto maybe_obstacle_index = wombat_core::find_if_raytrace(
    *start_pose_coord,
    *end_pose_coord,
    map.info,
    [&map, &last_free_index](wombat_core::grid_index_t index) {
      bool is_obstacle = map.data[index] > 0;
      if (!is_obstacle) {
        last_free_index = index;
      }
      return is_obstacle;
    });

  // If we didn't find any obstacle, just return the end pose
  if (!maybe_obstacle_index) {
    return end_pose;
  }

  // We found an obstacle between the start and the end poses.
  // Select the last free cell as the new pose
  // to simulate that the robot is now in contact with the obstacle.
  auto maybe_last_free_coord = wombat_core::grid_index_to_coord(*last_free_index, map.info);
  if (!maybe_last_free_coord) {
    return start_pose;
  }

  // We can't directly select the position corresponding to the last free cell, because,
  // due to the discrete values of the grid, it can cause instability in the robot pose.
  // For example the position could snap back and forth between two neighbor grid cell centers.
  // At the very least, we should ensure that we never move backward wrt the old pose.
  // The current implementation uses an interpolation to translate the grid motion into
  // world motion.
  auto interpolated_pose = end_pose;
  double x_scaling_factor =
    static_cast<double>(maybe_last_free_coord->x - start_pose_coord->x) / static_cast<double>(end_pose_coord->x);
  double y_scaling_factor =
    static_cast<double>(maybe_last_free_coord->y - start_pose_coord->y) / static_cast<double>(end_pose_coord->y);
  interpolated_pose.position.x = wombat_core::linear_interpolation(
    start_pose.position.x,
    end_pose.position.x,
    x_scaling_factor);
  interpolated_pose.position.y = wombat_core::linear_interpolation(
    start_pose.position.y,
    end_pose.position.y,
    y_scaling_factor);

  return interpolated_pose;
}

}  // namespace kennel
