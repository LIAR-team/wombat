// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kennel/mobile_base/ground_truth_manager.hpp"
#include "kennel/mobile_base/slam_manager.hpp"

#include "kennel/mobile_base/static_collision_manager.hpp"

namespace kennel
{

/**
 * @brief Class responsible for managing the movements and state
 * estimation of a mobile robot.
 */
class MobileBase
{
public:
  explicit MobileBase(rclcpp::Node * parent_node);

  localization_data_t get_ground_truth_data();

private:
  void mobile_base_update();

  std::optional<std::vector<geometry_msgs::msg::TransformStamped>>
  process_transforms(const std::vector<geometry_msgs::msg::TransformStamped> & tfs);

  bool setup_ground_truth();

  bool setup_slam();

  rclcpp::Node * m_parent_node;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
  rclcpp::Time m_last_cmd_time;
  geometry_msgs::msg::TwistStamped m_last_cmd_vel;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_ground_truth_map_sub;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr m_gt_map;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_slam_map_pub;

  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::vector<geometry_msgs::msg::TransformStamped> m_last_transforms;

  rclcpp::TimerBase::SharedPtr m_update_timer;

  std::unique_ptr<GroundTruthManager> m_gt_manager;
  std::unique_ptr<SlamManager> m_slam_manager;

  std::mutex m_mutex;
  rclcpp::Logger m_logger;

  std::shared_ptr<StaticCollisionManager> m_collision_manager;
};

}  // namespace kennel
