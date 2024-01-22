// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "kennel/mobile_base/slam_manager.hpp"

namespace kennel {

SlamManager::SlamManager(
  rclcpp::Node * parent_node,
  const rclcpp::Duration & update_period,
  const std::string & slam_frame_id)
: m_clock(parent_node->get_clock()), m_logger(parent_node->get_logger()),
  m_update_period(update_period), m_slam_frame_id(slam_frame_id)
{
  m_last_update_time = m_clock->now();
  RCLCPP_INFO(m_logger, "SLAM manager constructed");
}

std::optional<SlamManager::SlamData> SlamManager::slam_update(
  const geometry_msgs::msg::TransformStamped & gt_T_base,
  const nav_msgs::msg::OccupancyGrid & gt_map)
{
  const auto now = m_clock->now();
  if (now - m_last_update_time < m_update_period) {
    return std::nullopt;
  }

  m_last_update_time = now;

  // Naive implementation of SLAM: forward ground truth data
  SlamData data;
  data.map_T_base = gt_T_base;
  data.map_T_base.header.frame_id = m_slam_frame_id;
  data.map = gt_map;

  return data;
}

}
