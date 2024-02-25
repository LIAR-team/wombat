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
  InflatableCostmap(
    double robot_radius,
    const std::string & frame_id = "map");

  /**
   * @brief Update the occupancy representation of the costmap from a new whole map.
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr new_map);

  /**
   * @brief Update the occupancy representation of the costmap from a map update.
   * @param update sub-map to update in the costmap.
   */
  void update_map(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  /**
   * @brief Render the costmap using the latest available information
   * @return true if the rendering was successful
   */
  bool render_costmap();

  /**
   * @brief Get the latest non-inflated costmap.
   * @return nav2_costmap_2d::Costmap2D* the inflated costmap
   */
  const nav2_costmap_2d::Costmap2D * get_base_costmap();

  /**
   * @brief Get the latest rendered inflated costmap.
   * @return nav2_costmap_2d::Costmap2D* the inflated costmap
   */
  nav2_costmap_2d::Costmap2D * get_inflated_costmap();

private:
  std::shared_ptr<wombat_core::StaticLayer> m_static_layer;
  std::shared_ptr<wombat_core::InflationLayer> m_inflation_layer;

  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> m_layered_costmap;
};

}  // namespace wombat_core
