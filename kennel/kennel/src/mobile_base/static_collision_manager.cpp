// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include "wombat_core/costmap/inflatable_costmap.hpp"
#include "kennel/mobile_base/static_collision_manager.hpp"

namespace kennel
{

StaticCollisionManager::StaticCollisionManager(
  const nav_msgs::msg::OccupancyGrid & static_occ_grid)
{
  m_costmap = std::make_unique<wombat_core::InflatableCostmap>();
  m_costmap->update_map(std::make_shared<nav_msgs::msg::OccupancyGrid>(static_occ_grid));
  m_costmap->render_costmap();


  m_cost_translation_table = new char[256];

  // special values:
  m_cost_translation_table[0] = 0;  // NO obstacle
  m_cost_translation_table[253] = 99;  // INSCRIBED obstacle
  m_cost_translation_table[254] = 100;  // LETHAL obstacle
  m_cost_translation_table[255] = -1;  // UNKNOWN

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i < 253; i++) {
    m_cost_translation_table[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
  }

  m_node = std::make_shared<rclcpp::Node>("collision_node");

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  m_costmap_pub = m_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "costmap",
    custom_qos);

  m_costmap_pub->publish(std::move(this->prepare_grid()));
}

std::unique_ptr<nav_msgs::msg::OccupancyGrid> StaticCollisionManager::prepare_grid()
{
  auto costmap = m_costmap->get_inflated_costmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  auto grid_resolution = costmap->getResolution();
  auto grid_width = costmap->getSizeInCellsX();
  auto grid_height = costmap->getSizeInCellsY();

  auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  grid->header.frame_id = "ground_truth";
  //grid->header.stamp = clock_->now();

  grid->info.resolution = grid_resolution;

  grid->info.width = grid_width;
  grid->info.height = grid_height;

  double wx = 0.0;
  double wy = 0.0;
  costmap->mapToWorld(0, 0, wx, wy);
  grid->info.origin.position.x = wx - grid_resolution / 2;
  grid->info.origin.position.y = wy - grid_resolution / 2;
  grid->info.origin.position.z = 0.0;
  grid->info.origin.orientation.w = 1.0;

  grid->data.resize(grid->info.width * grid->info.height);

  unsigned char * data = costmap->getCharMap();
  for (unsigned int i = 0; i < grid->data.size(); i++) {
    grid->data[i] = m_cost_translation_table[data[i]];
  }

  return grid;
}


}  // namespace kennel
