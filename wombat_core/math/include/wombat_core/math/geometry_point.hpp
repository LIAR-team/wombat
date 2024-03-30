// Copyright 2021-2022 Soragna Alberto.

#pragma once

#include <cmath>

#include "geometry_msgs/msg/point.hpp"

namespace wombat_core
{

/**
 * @brief Compute the squared 2d distance (x, y) between the two points
 * @param p1 first point
 * @param p2 second point
 * @return constexpr double squared 2d distance
 */
constexpr double points_squared_distance_2d(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2) noexcept
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;

  return dx * dx + dy * dy;
}

/**
 * @brief Compute the 2d distance (x, y) between the two points
 * @param p1 first point
 * @param p2 second point
 * @return constexpr double 2d distance
 */
constexpr double points_distance_2d(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(points_squared_distance_2d(p1, p2));
}

/**
 * @brief Compute the squared 3d distance (x, y, z) between the two points
 * @param p1 first point
 * @param p2 second point
 * @return constexpr double squared 3d distance
 */
constexpr double points_squared_distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2) noexcept
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;

  return dx * dx + dy * dy + dz * dz;
}

/**
 * @brief Compute the 3d distance (x, y, z) between the two points
 * @param p1 first point
 * @param p2 second point
 * @return constexpr double 3d distance
 */
constexpr double points_distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(points_squared_distance(p1, p2));
}

}  // namespace wombat_core
