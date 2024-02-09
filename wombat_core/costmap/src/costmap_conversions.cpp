// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <array>

#include "wombat_core/costmap/costmap_conversions.hpp"
#include "wombat_core/costmap/costmap_utils.hpp"

namespace wombat_core
{

static std::array<char, 256> create_cost_translation_table()
{
  std::array<char, 256> cost_translation_table;
  cost_translation_table[0] = occupancy::FREE;
  cost_translation_table[253] = occupancy::INSCRIBED_OBS;
  cost_translation_table[254] = occupancy::LETHAL_OBS;
  cost_translation_table[255] = occupancy::UNKNOWN;

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i < 253; i++) {
    cost_translation_table[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
  }

  return cost_translation_table;
}

void costmap_to_occupancy_grid_values(
  nav2_costmap_2d::Costmap2D & costmap,
  nav_msgs::msg::OccupancyGrid & grid)
{
  // static initialization of the cost translation table.
  // it's thread-safe: https://stackoverflow.com/q/1661529/7108533
  static std::array<char, 256> s_cost_translation_table = create_cost_translation_table();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());

  // Write grid info
  float grid_resolution = costmap.getResolution();
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
  for (unsigned int i = 0; i < grid.data.size(); i++) {
    grid.data[i] = s_cost_translation_table[data[i]];
  }
}

}  // namespace wombat_core
