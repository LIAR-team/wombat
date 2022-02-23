// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#pragma once

#include <memory>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace wombat_strategy
{

static inline geometry_msgs::msg::Point index_to_world(
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

static inline unsigned int world_to_index(
  const geometry_msgs::msg::Point & world,
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  costmap->worldToMap(world.x, world.y, mx, my);
  return costmap->getIndex(mx, my);
}

static inline double points_squared_distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;

  return dx * dx + dy * dy;
}

static inline double points_distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  return sqrt(points_squared_distance(p1, p2));
}

}  // namespace wombat_strategy
