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
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kennel/common/plugin_interface/positioner_interface.hpp"
#include "kennel/mobile_base/ground_truth_manager.hpp"

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

  void on_gt_map_received(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  bool setup_ground_truth();

  bool load_positioner_plugins(rclcpp::Node * parent_node);

  rclcpp::Node * m_parent_node;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
  rclcpp::Time m_last_cmd_time;
  geometry_msgs::msg::TwistStamped m_last_cmd_vel;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_ground_truth_map_sub;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr m_gt_map;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_gt_costmap_pub;

  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::vector<geometry_msgs::msg::TransformStamped> m_last_transforms;

  rclcpp::TimerBase::SharedPtr m_update_timer;

  std::unique_ptr<GroundTruthManager> m_gt_manager;
  std::vector<std::shared_ptr<PositionerInterface>> m_positioners;

  pluginlib::ClassLoader<PositionerInterface> m_plugin_loader {
    "kennel",
    "kennel::PositionerInterface"};

  std::mutex m_mutex;
  rclcpp::Logger m_logger;
  rclcpp::Clock::SharedPtr m_clock;
};

}  // namespace kennel
