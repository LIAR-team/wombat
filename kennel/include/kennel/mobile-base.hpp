// Copyright 2024 Soragna Alberto.
// All Rights Reserved.

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kennel/ground-truth-manager.hpp"
#include "kennel/slam-manager.hpp"

class MobileBase
{
public:
  MobileBase(rclcpp::Node * parent_node);

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
  rclcpp::Logger m_logger;
};
