// Copyright 2024 Soragna Alberto.

#pragma once

#include <cstdint>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace wombat_core
{

namespace occupancy
{

constexpr int8_t FREE = 0;
constexpr int8_t INSCRIBED_OBS = 99;
constexpr int8_t LETHAL_OBS = 100;
constexpr int8_t UNKNOWN = -1;

}  // namespace occupancy

namespace costmap
{

constexpr unsigned char NO_INFORMATION = 255;
constexpr unsigned char LETHAL_OBSTACLE = 254;
constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
constexpr unsigned char MAX_NON_OBSTACLE = 252;
constexpr unsigned char FREE_SPACE = 0;

}  // namespace costmap

/**
 * @brief Converts a single costmap value into the corresponding
 * occupancy grid value.
 * @param value The costmap value to interpret
 * @return int8_t The interpreted occupancy grid value
 */
int8_t interpret_costmap_to_occupancy_value(unsigned char value);

/**
 * @brief Converts a single occupancy grid value into the corresponding
 * costmap value.
 * @param value The occupancy grid value to interpret
 * @param trinary_costmap If true, the costmap will treat all occupancy
 * values below the lethal threshold as free space.
 * If false they will be appropriately scaled to represent different costs.
 * @param unknown_cost_value occupancy value denoting unknown space
 * @param track_unknown_space if true the unknown space will be highlighted
 * in the costmap, if false it will be treated as free space.
 * @param lethal_threshold occupancy values above this threshold
 * are considered a lethal obstacle
 * @return unsigned char The interpreted costmap value
 */
unsigned char interpret_occupancy_to_costmap_value(
  unsigned char value,
  bool trinary_costmap,
  unsigned char unknown_cost_value,
  bool track_unknown_space,
  unsigned char lethal_threshold);

/**
 * @brief Writes an occupancy grid message from a costmap object.
 * The function will use thread-safe access to the costmap, locking it
 * while using it.
 * @note This function will not touch the header field of the grid msg
 * @param costmap costmap to read from
 * @param grid occupancy grid msgs to write into
 */
void costmap_to_occupancy_grid_values(
  nav2_costmap_2d::Costmap2D & costmap,
  nav_msgs::msg::OccupancyGrid & grid);

}  // namespace wombat_core
