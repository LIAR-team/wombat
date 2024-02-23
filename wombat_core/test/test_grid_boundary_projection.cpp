// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <functional>
#include <vector>

#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/raytrace.hpp"

static bool grid_coord_is_boundary(
  const wombat_core::grid_coord_t & coord,
  const wombat_core::MapMetaDataAdapter & map_info)
{
  // A grid coordinate is on the boundary if either one of its
  // scalar values is zero or its equal to the corresponding grid size - 1
  return
    coord.x() == 0 || coord.y() == 0 ||
    coord.x() == (map_info.grid_size.x() - 1) || coord.y() == (map_info.grid_size.y() - 1);
}

TEST(TestGridBoundaryProjection, StraightLines)
{
  wombat_core::MapMetaDataAdapter map_info;
  map_info.resolution = 0.02;
  map_info.grid_size = {1086, 443};

  double this_angle = -0.0314159;
  auto maybe_boundary_coord = wombat_core::project_to_grid_boundary(
    wombat_core::grid_coord_t{50, 50},
    map_info,
    this_angle);
  ASSERT_NE(std::nullopt, maybe_boundary_coord);
  ASSERT_TRUE(wombat_core::grid_coord_is_valid(*maybe_boundary_coord, map_info));
  EXPECT_TRUE(grid_coord_is_boundary(*maybe_boundary_coord, map_info));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
