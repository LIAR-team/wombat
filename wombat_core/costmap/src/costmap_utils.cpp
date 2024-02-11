// Copyright 2021-2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "wombat_core/costmap/costmap_utils.hpp"

namespace wombat_core
{

geometry_msgs::msg::Point index_to_world(
  unsigned int index,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  costmap->indexToCells(index, mx, my);
  geometry_msgs::msg::Point world;
  costmap->mapToWorld(mx, my, world.x, world.y);
  return world;
}

unsigned int world_to_index(
  const geometry_msgs::msg::Point & world,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  costmap->worldToMap(world.x, world.y, mx, my);
  return costmap->getIndex(mx, my);
}

size_t costmap_size(const nav2_costmap_2d::Costmap2D & costmap)
{
  return static_cast<size_t>(costmap.getSizeInCellsX()) * static_cast<size_t>(costmap.getSizeInCellsY());
}

}  // namespace wombat_core
