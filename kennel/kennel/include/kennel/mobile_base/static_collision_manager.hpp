// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>

#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "wombat_core/costmap/inflatable_costmap.hpp"

namespace kennel
{

/**
 * @brief Class responsible for keeping track of the ground truth
 * pose of a mobile robot.
 */
class StaticCollisionManager
{
public:
  explicit StaticCollisionManager(
    const nav_msgs::msg::OccupancyGrid & static_occ_grid);

private:
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> prepare_grid();

  std::unique_ptr<wombat_core::InflatableCostmap> m_costmap;

  std::shared_ptr<rclcpp::Node> m_node;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> m_costmap_pub;

  char * m_cost_translation_table;
};

}  // namespace kennel
