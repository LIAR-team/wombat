// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <functional>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include "wombat_core/math/grid/coordinates.hpp"
#include "wombat_core/math/grid/neighbors.hpp"

TEST(TestGridNeighbors, ZeroDimGrid)
{
  size_t count = 0;
  auto count_func = [&count](wombat_core::grid_index_t) {
    count++;
    return false;
  };

  // Test grids where one or both dimensions are zero
  wombat_core::for_each_grid_neighbor(
    wombat_core::grid_index_t(0),
    10,
    0,
    count_func,
    true);
  ASSERT_EQ(count, 0);

  wombat_core::for_each_grid_neighbor(
    wombat_core::grid_index_t(0),
    0,
    4,
    count_func,
    true);
  ASSERT_EQ(count, 0);

  wombat_core::for_each_grid_neighbor(
    wombat_core::grid_index_t(0),
    0,
    0,
    count_func,
    true);
  ASSERT_EQ(count, 0);
}

TEST(TestGridNeighbors, TooSmallGrid)
{
  size_t count = 0;
  auto count_func = [&count](wombat_core::grid_index_t) {
    count++;
    return false;
  };

  // Test valid grids where the index point is invalid
  wombat_core::for_each_grid_neighbor(
    wombat_core::grid_index_t(100),
    10,
    10,
    count_func,
    true);
  ASSERT_EQ(count, 0);

  wombat_core::for_each_grid_neighbor(
    wombat_core::grid_index_t(101),
    10,
    10,
    count_func,
    true);
  ASSERT_EQ(count, 0);

  wombat_core::for_each_grid_neighbor(
    wombat_core::grid_index_t(-1),
    10,
    10,
    count_func,
    true);
  ASSERT_EQ(count, 0);
}

// Note: we could use gmock matchers for this, but when enabled via `ament_add_gmock`
// this triggers some warnings (which are treated as errors and fail the build)
static void expect_contains_grid_coord(
  const std::vector<wombat_core::grid_coord_t> & coords,
  const wombat_core::grid_coord_t & matching_coord)
{
  for (auto c : coords) {
    if (c.x == matching_coord.x && c.y == matching_coord.y) {
      return;
    }
  }
  FAIL() << "Not found [" << matching_coord.x << " " << matching_coord.y << "]" << std::endl;
}

TEST(TestGridNeighbors, CheckNeighbors)
{
  nav_msgs::msg::MapMetaData map_info;
  map_info.width = 10;
  map_info.height = 10;

  std::vector<wombat_core::grid_coord_t> coords;
  auto store_func = [&coords, &map_info](wombat_core::grid_index_t i) {
    auto this_coord = wombat_core::grid_index_to_coord(i, map_info);
    EXPECT_NE(std::nullopt, this_coord);
    if (this_coord) {
      coords.push_back(*this_coord);
    }
    return false;
  };

  coords.clear();
  wombat_core::for_each_grid_neighbor(
    wombat_core::grid_index_t(0),
    map_info.width,
    map_info.height,
    store_func,
    false);
  ASSERT_EQ(coords.size(), 2u);
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{1, 0});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{0, 1});

  coords.clear();
  wombat_core::for_each_grid_neighbor(
    wombat_core::grid_index_t(0),
    map_info.width,
    map_info.height,
    store_func,
    true);
  ASSERT_EQ(coords.size(), 3u);
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{1, 0});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{0, 1});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{1, 1});

  coords.clear();
  wombat_core::for_each_grid_neighbor(
    *(wombat_core::grid_coord_to_index(wombat_core::grid_coord_t{5, 5}, map_info)),
    map_info.width,
    map_info.height,
    store_func,
    false);
  ASSERT_EQ(coords.size(), 4u);
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{5, 6});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{6, 5});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{5, 4});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{4, 5});

  coords.clear();
  wombat_core::for_each_grid_neighbor(
    *(wombat_core::grid_coord_to_index(wombat_core::grid_coord_t{5, 5}, map_info)),
    map_info.width,
    map_info.height,
    store_func,
    true);
  ASSERT_EQ(coords.size(), 8u);
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{5, 6});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{6, 5});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{5, 4});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{4, 5});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{6, 6});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{6, 4});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{4, 6});
  expect_contains_grid_coord(coords, wombat_core::grid_coord_t{4, 4});
}

TEST(TestGridNeighbors, TerminateEarly)
{
  nav_msgs::msg::MapMetaData map_info;
  map_info.width = 10;
  map_info.height = 10;

  size_t count = 0;
  size_t limit = 0;
  auto count_func = [&count, &limit](wombat_core::grid_index_t) {
    count++;
    return count >= limit;
  };

  for (size_t i = 1; i <= 8; i++) {
    count = 0;
    limit = i;
    wombat_core::for_each_grid_neighbor(
      *(wombat_core::grid_coord_to_index(wombat_core::grid_coord_t{5, 5}, map_info)),
      map_info.width,
      map_info.height,
      count_func,
      true);
    ASSERT_EQ(count, limit);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
