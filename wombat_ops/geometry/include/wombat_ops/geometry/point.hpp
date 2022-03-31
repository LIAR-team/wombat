// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#pragma once

#include <cmath>

#include "geometry_msgs/msg/point.hpp"

namespace wombat_ops
{

/**
 * @brief Squared distance between two arbitrary points.
 */
static inline double points_squared_distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;

  return dx * dx + dy * dy;
}

/**
 * @brief Distance between two arbitrary points.
 */
static inline double points_distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  return sqrt(points_squared_distance(p1, p2));
}

}  // namespace wombat_ops
