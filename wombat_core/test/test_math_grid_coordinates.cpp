// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include "wombat_core/grid/coordinates.hpp"

using wombat_core::grid_coord_t;
using wombat_core::grid_index_t;
using wombat_core::MapMetaDataAdapter;

static geometry_msgs::msg::Point make_pt(double x = 0.0, double y = 0.0)
{
  geometry_msgs::msg::Point pt;
  pt.x = x;
  pt.y = y;
  return pt;
}

static MapMetaDataAdapter make_map_info(
  double resolution = 0.0,
  int width = 0,
  int height = 0,
  const geometry_msgs::msg::Point & origin = geometry_msgs::msg::Point())
{
  MapMetaDataAdapter map_info;
  map_info.resolution = resolution;
  map_info.grid_size = wombat_core::grid_size_t{
    width,
    height
  };
  map_info.origin.position = origin;
  return map_info;
}

TEST(TestGrid, ConversionsInGridTooSmall)
{
  auto map_info = make_map_info(0.1, 2, 2);

  EXPECT_NE(std::nullopt, wombat_core::grid_coord_to_index(grid_coord_t{1, 0}, map_info));
  EXPECT_EQ(std::nullopt, wombat_core::grid_coord_to_index(grid_coord_t{1, 4}, map_info));
  EXPECT_EQ(std::nullopt, wombat_core::grid_coord_to_index(grid_coord_t{-1, 0}, map_info));
  EXPECT_EQ(std::nullopt, wombat_core::grid_coord_to_index(grid_coord_t{1, -4}, map_info));

  EXPECT_NE(std::nullopt, wombat_core::grid_index_to_coord(3, map_info));
  EXPECT_EQ(std::nullopt, wombat_core::grid_index_to_coord(4, map_info));

  EXPECT_NE(std::nullopt, wombat_core::world_pt_to_grid_coord(make_pt(0.1, 0.1), map_info));
  EXPECT_EQ(std::nullopt, wombat_core::world_pt_to_grid_coord(make_pt(-13.1, 0.1), map_info));
  EXPECT_EQ(std::nullopt, wombat_core::world_pt_to_grid_coord(make_pt(2.4, 0.1), map_info));
  EXPECT_EQ(std::nullopt, wombat_core::world_pt_to_grid_coord(make_pt(0.1, 0.25), map_info));

  EXPECT_NE(std::nullopt, wombat_core::grid_coord_to_world_pt(grid_coord_t{0, 1}, map_info));
  EXPECT_EQ(std::nullopt, wombat_core::grid_coord_to_world_pt(grid_coord_t{55, 112}, map_info));
  EXPECT_EQ(std::nullopt, wombat_core::grid_coord_to_world_pt(grid_coord_t{-55, 112}, map_info));
  EXPECT_EQ(std::nullopt, wombat_core::grid_coord_to_world_pt(grid_coord_t{55, -112}, map_info));

  EXPECT_NE(std::nullopt, wombat_core::world_pt_to_grid_index(make_pt(0.1, 0.1), map_info));
  EXPECT_EQ(std::nullopt, wombat_core::world_pt_to_grid_index(make_pt(-13.1, 0.1), map_info));

  std::cout<<"LINE 63 "<<std::endl;
  EXPECT_NE(std::nullopt, wombat_core::grid_index_to_world_pt(3, map_info));
  EXPECT_EQ(std::nullopt, wombat_core::grid_index_to_world_pt(4, map_info));
}

struct grid_conversion_data_t
{
  MapMetaDataAdapter map_info;
  grid_index_t grid_index;
  grid_coord_t grid_coord;
  geometry_msgs::msg::Point world_pt;
};

static MapMetaDataAdapter s_map_10x10 = make_map_info(0.1, 10, 10);
static MapMetaDataAdapter s_map_10x10_offset_gt0 = make_map_info(0.1, 10, 10, make_pt(1.0, 1.0));
static MapMetaDataAdapter s_map_10x10_offset_lt0 = make_map_info(0.1, 10, 10, make_pt(-0.5, -0.25));

class TestConversionsWithParams
  : public testing::TestWithParam<grid_conversion_data_t>
{};

INSTANTIATE_TEST_SUITE_P(
  TestGridP,
  TestConversionsWithParams,
  ::testing::Values(
    grid_conversion_data_t{s_map_10x10, 0, {0, 0}, make_pt(0.05, 0.05)},
    grid_conversion_data_t{s_map_10x10, 3, {3, 0}, make_pt(0.35, 0.05)},
    grid_conversion_data_t{s_map_10x10, 99, {9, 9}, make_pt(0.95, 0.95)},
    grid_conversion_data_t{s_map_10x10_offset_gt0, 0, {0, 0}, make_pt(1.05, 1.05)},
    grid_conversion_data_t{s_map_10x10_offset_gt0, 3, {3, 0}, make_pt(1.35, 1.05)},
    grid_conversion_data_t{s_map_10x10_offset_gt0, 99, {9, 9}, make_pt(1.95, 1.95)},
    grid_conversion_data_t{s_map_10x10_offset_lt0, 0, {0, 0}, make_pt(-0.45, -0.2)},
    grid_conversion_data_t{s_map_10x10_offset_lt0, 3, {3, 0}, make_pt(-0.15, -0.2)},
    grid_conversion_data_t{s_map_10x10_offset_lt0, 99, {9, 9}, make_pt(0.45, 0.7)}
));

TEST_P(TestConversionsWithParams, ValidConversionsAndBack)
{
  auto test_data = GetParam();

  static constexpr double EPSILON = 0.001;

  // Coord2D to grid coord
  auto grid_coord = wombat_core::grid_index_to_coord(test_data.grid_index, test_data.map_info);
  ASSERT_NE(std::nullopt, grid_coord);
  EXPECT_EQ(grid_coord->x(), test_data.grid_coord.x());
  EXPECT_EQ(grid_coord->y(), test_data.grid_coord.y());
  // Grid coord back to index
  auto grid_index = wombat_core::grid_coord_to_index(*grid_coord, test_data.map_info);
  ASSERT_NE(std::nullopt, grid_index);
  EXPECT_EQ(*grid_index, test_data.grid_index);

  // Coord2D to world pt
  auto world_pt = wombat_core::grid_index_to_world_pt(test_data.grid_index, test_data.map_info);
  ASSERT_NE(std::nullopt, world_pt);
  EXPECT_NEAR(world_pt->x, test_data.world_pt.x, EPSILON);
  EXPECT_NEAR(world_pt->y, test_data.world_pt.y, EPSILON);
  EXPECT_NEAR(world_pt->z, test_data.world_pt.z, EPSILON);
  // World pt back to index
  grid_index = wombat_core::world_pt_to_grid_index(*world_pt, test_data.map_info);
  ASSERT_NE(std::nullopt, grid_index);
  EXPECT_EQ(*grid_index, test_data.grid_index);

  // World pt to grid coord
  grid_coord = wombat_core::world_pt_to_grid_coord(test_data.world_pt, test_data.map_info);
  ASSERT_NE(std::nullopt, grid_coord);
  EXPECT_EQ(grid_coord->x(), test_data.grid_coord.x());
  EXPECT_EQ(grid_coord->y(), test_data.grid_coord.y());
  // Grid coord back to world pt
  world_pt = wombat_core::grid_coord_to_world_pt(*grid_coord, test_data.map_info);
  ASSERT_NE(std::nullopt, world_pt);
  EXPECT_NEAR(world_pt->x, test_data.world_pt.x, EPSILON);
  EXPECT_NEAR(world_pt->y, test_data.world_pt.y, EPSILON);
  EXPECT_NEAR(world_pt->z, test_data.world_pt.z, EPSILON);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
