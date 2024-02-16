// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "kennel/mobile_base/slam_manager.hpp"

namespace kennel
{

SlamManager::SlamManager(
  rclcpp::Node * parent_node,
  const rclcpp::Duration & update_period,
  std::string slam_frame_id)
: m_clock(parent_node->get_clock()), m_logger(parent_node->get_logger()),
  m_update_period(update_period), m_slam_frame_id(std::move(slam_frame_id))
{
  m_last_update_time = m_clock->now();
  RCLCPP_INFO(m_logger, "SLAM manager constructed");
}

std::optional<localization_data_t> SlamManager::slam_update(
  const localization_data_t & gt_data)
{
  const auto now = m_clock->now();
  if (now - m_last_update_time < m_update_period) {
    return std::nullopt;
  }

  m_last_update_time = now;

  // Naive implementation of SLAM: forward ground truth data
  localization_data_t slam_data;
  slam_data.robot_pose = gt_data.robot_pose;
  slam_data.robot_pose.header.frame_id = m_slam_frame_id;
  auto slam_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*(gt_data.map));
  slam_map->header.frame_id = m_slam_frame_id;
  slam_data.map = slam_map;

  return slam_data;
}

}  // namespace kennel
