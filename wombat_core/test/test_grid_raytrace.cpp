// Copyright 2024 Soragna Alberto.

#include <gtest/gtest.h>

#include <functional>
#include <vector>

#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/raytrace.hpp"

using wombat_core::MapMetaDataAdapter;

TEST(TestRaytrace, StraightLines)
{
  MapMetaDataAdapter map_info;
  map_info.resolution = 1.0;
  map_info.grid_size = {10, 10};

  // Horizontal line to the right
  size_t count = 0;
  auto maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{0, 4},
    wombat_core::grid_coord_t{9, 4},
    map_info,
    [&count, &map_info](wombat_core::grid_index_t idx) {
      auto maybe_coord = wombat_core::grid_index_to_coord(idx, map_info);
      EXPECT_NE(maybe_coord, std::nullopt);
      EXPECT_EQ(maybe_coord->x(), count);
      EXPECT_EQ(maybe_coord->y(), 4);
      count++;
      return false;
    });
  EXPECT_EQ(count, 10);
  EXPECT_EQ(maybe_found_index, std::nullopt);

  // Horizontal line to the left
  count = 0;
  maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{7, 9},
    wombat_core::grid_coord_t{1, 9},
    map_info,
    [&count, &map_info](wombat_core::grid_index_t idx) {
      auto maybe_coord = wombat_core::grid_index_to_coord(idx, map_info);
      EXPECT_NE(maybe_coord, std::nullopt);
      EXPECT_EQ(maybe_coord->x(), 7 - count);
      EXPECT_EQ(maybe_coord->y(), 9);
      count++;
      return false;
    });
  EXPECT_EQ(count, 7);
  EXPECT_EQ(maybe_found_index, std::nullopt);

  // Vertical line to the top
  count = 0;
  maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{0, 0},
    wombat_core::grid_coord_t{0, 9},
    map_info,
    [&count, &map_info](wombat_core::grid_index_t idx) {
      auto maybe_coord = wombat_core::grid_index_to_coord(idx, map_info);
      EXPECT_NE(maybe_coord, std::nullopt);
      EXPECT_EQ(maybe_coord->x(), 0);
      EXPECT_EQ(maybe_coord->y(), count);
      count++;
      return false;
    });
  EXPECT_EQ(count, 10);
  EXPECT_EQ(maybe_found_index, std::nullopt);

  // Vertical line to the bottom
  count = 0;
  maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{9, 5},
    wombat_core::grid_coord_t{9, 0},
    map_info,
    [&count, &map_info](wombat_core::grid_index_t idx) {
      auto maybe_coord = wombat_core::grid_index_to_coord(idx, map_info);
      EXPECT_NE(maybe_coord, std::nullopt);
      EXPECT_EQ(maybe_coord->x(), 9);
      EXPECT_EQ(maybe_coord->y(), 5 - count);
      count++;
      return false;
    });
  EXPECT_EQ(count, 6);
  EXPECT_EQ(maybe_found_index, std::nullopt);
}

TEST(TestRaytrace, DiagonalLines)
{
  MapMetaDataAdapter map_info;
  map_info.resolution = 1.0;
  map_info.grid_size = {10, 10};

  // Diagonal line at 45 degrees
  size_t count = 0;
  auto maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{0, 0},
    wombat_core::grid_coord_t{9, 9},
    map_info,
    [&count, &map_info](wombat_core::grid_index_t idx) {
      auto maybe_coord = wombat_core::grid_index_to_coord(idx, map_info);
      EXPECT_NE(maybe_coord, std::nullopt);
      EXPECT_EQ(maybe_coord->x(), count);
      EXPECT_EQ(maybe_coord->y(), count);
      count++;
      return false;
    });
  EXPECT_EQ(count, 10);
  EXPECT_EQ(maybe_found_index, std::nullopt);
}

TEST(TestRaytrace, FoundItem)
{
  MapMetaDataAdapter map_info;
  map_info.resolution = 1.0;
  map_info.grid_size = {10, 10};

  size_t stop_count = 1;
  size_t count = 0;

  auto predicate = [&count, &stop_count](wombat_core::grid_index_t idx) {
      (void)idx;
      count++;
      return count == stop_count;
    };

  auto maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{0, 0},
    wombat_core::grid_coord_t{5, 0},
    map_info,
    predicate);
  EXPECT_EQ(count, stop_count);
  EXPECT_NE(maybe_found_index, std::nullopt);
  auto maybe_found_coord = wombat_core::grid_index_to_coord(*maybe_found_index, map_info);
  EXPECT_NE(maybe_found_coord, std::nullopt);
  EXPECT_EQ(maybe_found_coord->x(), 0);
  EXPECT_EQ(maybe_found_coord->y(), 0);

  stop_count = 2;
  count = 0;
  maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{0, 0},
    wombat_core::grid_coord_t{5, 0},
    map_info,
    predicate);
  EXPECT_EQ(count, stop_count);
  EXPECT_NE(maybe_found_index, std::nullopt);
  maybe_found_coord = wombat_core::grid_index_to_coord(*maybe_found_index, map_info);
  EXPECT_NE(maybe_found_coord, std::nullopt);
  EXPECT_EQ(maybe_found_coord->x(), 1);
  EXPECT_EQ(maybe_found_coord->y(), 0);

  stop_count = 6;
  count = 0;
  maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{0, 0},
    wombat_core::grid_coord_t{5, 0},
    map_info,
    predicate);
  EXPECT_EQ(count, stop_count);
  EXPECT_NE(maybe_found_index, std::nullopt);
  maybe_found_coord = wombat_core::grid_index_to_coord(*maybe_found_index, map_info);
  EXPECT_NE(maybe_found_coord, std::nullopt);
  EXPECT_EQ(maybe_found_coord->x(), 5);
  EXPECT_EQ(maybe_found_coord->y(), 0);
}

TEST(TestRaytrace, OutOfBounds)
{
  MapMetaDataAdapter map_info;
  map_info.resolution = 1.0;
  map_info.grid_size = {10, 10};

  size_t count = 0;
  auto predicate = [&count](wombat_core::grid_index_t idx) {
      (void)idx;
      return false;
    };

  // End point exceeds width
  auto maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{0, 0},
    wombat_core::grid_coord_t{10, 0},
    map_info,
    predicate);
  EXPECT_EQ(count, 0);
  EXPECT_EQ(maybe_found_index, std::nullopt);

  // Start and end points exceed width
  maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{10, 3},
    wombat_core::grid_coord_t{12, 3},
    map_info,
    predicate);
  EXPECT_EQ(count, 0);
  EXPECT_EQ(maybe_found_index, std::nullopt);

  // End point exceeds height
  maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{4, std::numeric_limits<unsigned int>::max()},
    wombat_core::grid_coord_t{7, std::numeric_limits<unsigned int>::max()},
    map_info,
    predicate);
  EXPECT_EQ(count, 0);
  EXPECT_EQ(maybe_found_index, std::nullopt);
}

TEST(TestRaytrace, StartPastEnd)
{
  MapMetaDataAdapter map_info;
  map_info.resolution = 1.0;
  map_info.grid_size = {10, 10};

  size_t count = 0;
  auto predicate = [&count](wombat_core::grid_index_t idx) {
      (void)idx;
      return false;
    };

  // Start point is past end
  auto maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{5, 5},
    wombat_core::grid_coord_t{2, 5},
    map_info,
    predicate);
  EXPECT_EQ(count, 0);
  EXPECT_EQ(maybe_found_index, std::nullopt);

  // Start point is past end
  maybe_found_index = wombat_core::find_if_raytrace(
    wombat_core::grid_coord_t{4, 8},
    wombat_core::grid_coord_t{1, 2},
    map_info,
    predicate);
  EXPECT_EQ(count, 0);
  EXPECT_EQ(maybe_found_index, std::nullopt);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
