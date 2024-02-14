// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <array>

#include "wombat_core/costmap/costmap_conversions.hpp"
#include "wombat_core/costmap/costmap_utils.hpp"

namespace wombat_core
{

static std::array<int8_t, 256> create_cost_translation_table()
{
  static_assert(costmap::NO_INFORMATION == 255, "Unexpected NO_INFORMATION value");
  static_assert(costmap::LETHAL_OBSTACLE == 254, "Unexpected LETHAL_OBSTACLE value");
  static_assert(costmap::FREE_SPACE == 0, "Unexpected FREE_SPACE value");

  std::array<int8_t, 256> cost_translation_table;
  cost_translation_table[0] = occupancy::FREE;
  cost_translation_table[253] = occupancy::INSCRIBED_OBS;
  cost_translation_table[254] = occupancy::LETHAL_OBS;
  cost_translation_table[255] = occupancy::UNKNOWN;

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i < 253; i++) {
    cost_translation_table[i] = static_cast<int8_t>(1 + (97 * (i - 1)) / 251);
  }

  return cost_translation_table;
}

int8_t interpret_costmap_to_occupancy_value(unsigned char value)
{
  // static initialization of the cost translation table.
  // it's thread-safe: https://stackoverflow.com/q/1661529/7108533
  // This means that the first invocation of this function will be "slow"
  static std::array<int8_t, 256> s_cost_translation_table = create_cost_translation_table();

  return s_cost_translation_table[static_cast<size_t>(value)];
}

unsigned char interpret_occupancy_to_costmap_value(
  unsigned char value,
  bool trinary_costmap,
  unsigned char unknown_cost_value,
  bool track_unknown_space,
  unsigned char lethal_threshold)
{
  // Check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space && value == unknown_cost_value) {
    return costmap::NO_INFORMATION;
  } else if (!track_unknown_space && value == unknown_cost_value) {
    return costmap::FREE_SPACE;
  } else if (value >= lethal_threshold) {
    return costmap::LETHAL_OBSTACLE;
  }

  if (trinary_costmap) {
    return costmap::FREE_SPACE;
  }
  double scale = static_cast<double>(value) / lethal_threshold;
  return static_cast<unsigned char>(scale * costmap::LETHAL_OBSTACLE);
}

void costmap_to_occupancy_grid_values(
  nav2_costmap_2d::Costmap2D & costmap,
  nav_msgs::msg::OccupancyGrid & grid)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());

  // Write grid info
  auto grid_resolution = static_cast<float>(costmap.getResolution());
  grid.info.resolution = grid_resolution;
  grid.info.width = costmap.getSizeInCellsX();
  grid.info.height = costmap.getSizeInCellsY();
  double wx = 0.0;
  double wy = 0.0;
  costmap.mapToWorld(0, 0, wx, wy);
  grid.info.origin.position.x = wx - grid_resolution / 2;
  grid.info.origin.position.y = wy - grid_resolution / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // Write grid data
  grid.data.resize(costmap_size(costmap));
  auto data = costmap.getCharMap();
  for (size_t i = 0; i < grid.data.size(); i++) {
    grid.data[i] = interpret_costmap_to_occupancy_value(data[i]);
  }
}

}  // namespace wombat_core
