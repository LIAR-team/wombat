// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "wombat_core/costmap/inflatable_costmap.hpp"
#include "wombat_core/costmap/inflation_layer.hpp"
#include "wombat_core/costmap/static_layer.hpp"

namespace wombat_core
{

InflatableCostmap::InflatableCostmap()
{
  m_layered_costmap = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
    "scemotto", false, false);

  m_static_layer = std::make_shared<wombat_core::StaticLayer>();
  m_inflation_layer = std::make_shared<wombat_core::InflationLayer>();
  m_layered_costmap->addPlugin(m_static_layer);
  m_layered_costmap->addPlugin(m_inflation_layer);

  // Setup all plugins before doing anything
  m_static_layer->setup(m_layered_costmap.get());
  m_inflation_layer->setup(m_layered_costmap.get());
}

bool InflatableCostmap::render_costmap()
{
  m_layered_costmap->updateMap(0.0, 0.0, 0.0);
  return true;
}

void InflatableCostmap::update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr new_map)
{
  m_static_layer->update_map(new_map);
}

void InflatableCostmap::update_map(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  m_static_layer->update_map(update);
}

//const InflatableCostmap::nav2_costmap_2d::Costmap2D * get_base_costmap()
//{
//  return m_static_layer->get_costmap();
//}

nav2_costmap_2d::Costmap2D * InflatableCostmap::get_inflated_costmap()
{
  return m_layered_costmap->getCostmap();
}

}
