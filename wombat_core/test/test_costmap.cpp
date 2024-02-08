// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "wombat_core/costmap/costmap_utils.hpp"

TEST(test_costmap, convert_index_word)
{
  constexpr unsigned int GRID_WIDTH = 10;
  constexpr unsigned int GRID_HEIGHT = 10;
  constexpr double RESOLUTION = 0.5;
  constexpr double ORIGIN_X = 0.0;
  constexpr double ORIGIN_Y = 0.0;

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    GRID_WIDTH, GRID_HEIGHT, RESOLUTION, ORIGIN_X, ORIGIN_Y, 0.0);

  {
    auto cell_center = wombat_core::index_to_world(0, costmap);
    EXPECT_EQ(cell_center.x, 0.25);
    EXPECT_EQ(cell_center.y, 0.25);

    auto cell_index = wombat_core::world_to_index(cell_center, costmap);
    EXPECT_EQ(cell_index, 0u);
  }

  {
    auto cell_center = wombat_core::index_to_world(9, costmap);
    EXPECT_EQ(cell_center.x, 4.75);
    EXPECT_EQ(cell_center.y, 0.25);

    auto cell_index = wombat_core::world_to_index(cell_center, costmap);
    EXPECT_EQ(cell_index, 9u);
  }

  {
    geometry_msgs::msg::Point cell_center;
    cell_center.x = 2.75;
    cell_center.y = 2.75;

    auto cell_index = wombat_core::world_to_index(cell_center, costmap);
    EXPECT_EQ(cell_index, 55u);

    auto reconstructed_cell_center = wombat_core::index_to_world(cell_index, costmap);
    EXPECT_EQ(reconstructed_cell_center.x, cell_center.x);
    EXPECT_EQ(reconstructed_cell_center.y, cell_center.y);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
