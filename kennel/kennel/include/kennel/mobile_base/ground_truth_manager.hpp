// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kennel
{

/**
 * @brief Class responsible for keeping track of the ground truth
 * pose of a mobile robot.
 */
class GroundTruthManager
{
public:
  GroundTruthManager(
    rclcpp::Node * parent_node,
    std::string ground_truth_frame_id,
    const std::chrono::milliseconds & cmd_timeout,
    std::string robot_base_frame_id = "base_link");

  geometry_msgs::msg::TransformStamped pose_update(
    const geometry_msgs::msg::TwistStamped & cmd_vel,
    const nav_msgs::msg::OccupancyGrid & map);

  geometry_msgs::msg::TransformStamped get_pose() const;

  void reset_pose(const geometry_msgs::msg::Pose & new_pose);

private:
  geometry_msgs::msg::Pose apply_map_constraints(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Pose & old_pose,
    const geometry_msgs::msg::Pose & new_pose);

  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Logger m_logger;

  std::string m_ground_truth_frame_id {};
  std::string m_robot_base_frame_id {};
  std::chrono::milliseconds m_cmd_timeout {};

  geometry_msgs::msg::Pose m_gt_pose;
  geometry_msgs::msg::TransformStamped m_gt_transform;
  std::optional<rclcpp::Time> m_last_pose_update_time;
};

}  // namespace kennel
