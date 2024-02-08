// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "wombat_core/costmap/inflation_layer.hpp"
#include "wombat_core/costmap/static_layer.hpp"


namespace wombat_core
{

class InflatableCostmap
{
public:
  InflatableCostmap();

  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr new_map);

  /**
   * @brief Callback to update the costmap's map from the map_server (or SLAM)
   * with an update in a particular area of the map
   */
  void update_map(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  //const nav2_costmap_2d::Costmap2D * get_base_costmap();

  nav2_costmap_2d::Costmap2D * get_inflated_costmap();

  bool render_costmap();

private:
  std::shared_ptr<wombat_core::StaticLayer> m_static_layer;
  std::shared_ptr<wombat_core::InflationLayer> m_inflation_layer;

  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> m_layered_costmap;
};

}
