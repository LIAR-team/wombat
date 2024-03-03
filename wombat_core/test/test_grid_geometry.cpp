// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include "wombat_core/grid/geometry.hpp"

TEST(TestGridGeometry, PointInSegment)
{
  wombat_core::grid_coord_t pA {0, 0};
  wombat_core::grid_coord_t pB {1, 0};
  wombat_core::grid_coord_t pC {-1, 0};

  EXPECT_TRUE(wombat_core::point_in_segment({{pC, pA, pB}}));
  EXPECT_TRUE(wombat_core::point_in_segment({{pB, pA, pC}}));
  EXPECT_FALSE(wombat_core::point_in_segment({{pA, pB, pC}}));
  EXPECT_FALSE(wombat_core::point_in_segment({{pA, pC, pB}}));
  EXPECT_FALSE(wombat_core::point_in_segment({{pC, pB, pA}}));
  EXPECT_FALSE(wombat_core::point_in_segment({{pB, pC, pA}}));

  wombat_core::grid_coord_t pD {1, 1};
  wombat_core::grid_coord_t pE {2, 2};
  wombat_core::grid_coord_t pF {3, 3};

  EXPECT_TRUE(wombat_core::point_in_segment({{pA, pD, pE}}));
  EXPECT_TRUE(wombat_core::point_in_segment({{pE, pD, pA}}));
  EXPECT_FALSE(wombat_core::point_in_segment({{pD, pF, pE}}));

  wombat_core::grid_coord_t pG {0, -10};
  wombat_core::grid_coord_t pH {0, -20};

  EXPECT_TRUE(wombat_core::point_in_segment({{pH, pG, pA}}));
  EXPECT_TRUE(wombat_core::point_in_segment({{pA, pG, pH}}));
  EXPECT_FALSE(wombat_core::point_in_segment({{pG, pA, pH}}));
}

TEST(TestGridGeometry, PointInPolygonSquare)
{
  std::vector<wombat_core::grid_coord_t> polygon;
  polygon.push_back({-2, -2});
  polygon.push_back({-2, 2});
  polygon.push_back({2, 2});
  polygon.push_back({2, -2});

  // Points inside
  EXPECT_TRUE(wombat_core::point_in_polygon({0, 0}, polygon, true));
  EXPECT_TRUE(wombat_core::point_in_polygon({0, 0}, polygon, false));
  EXPECT_TRUE(wombat_core::point_in_polygon({1, 1}, polygon, true));
  EXPECT_TRUE(wombat_core::point_in_polygon({1, 1}, polygon, false));

  // Points on boundary
  // Right (max x) boundary
  EXPECT_TRUE(wombat_core::point_in_polygon({2, 0}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({2, 0}, polygon, false));
  // Top (max y) boundary
  EXPECT_TRUE(wombat_core::point_in_polygon({-1, 2}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({-1, 2}, polygon, false));
  // Left (min x) boundary
  EXPECT_TRUE(wombat_core::point_in_polygon({-2, 0}, polygon, true));
  EXPECT_TRUE(wombat_core::point_in_polygon({-2, 0}, polygon, false));
  // Bottom (min y) boundary
  EXPECT_TRUE(wombat_core::point_in_polygon({1, -2}, polygon, true));
  EXPECT_TRUE(wombat_core::point_in_polygon({1, -2}, polygon, false));

  // Vertices
  // Bottom left (min x min y) corner
  EXPECT_TRUE(wombat_core::point_in_polygon({-2, -2}, polygon, true));
  EXPECT_TRUE(wombat_core::point_in_polygon({-2, -2}, polygon, false));
  // Bottom right (max x min y) corner
  EXPECT_TRUE(wombat_core::point_in_polygon({2, -2}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({2, -2}, polygon, false));
  // Top left (min x max y) corner
  EXPECT_TRUE(wombat_core::point_in_polygon({-2, 2}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({-2, 2}, polygon, false));
  // Top right (max x max y) corner
  EXPECT_TRUE(wombat_core::point_in_polygon({2, 2}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({2, 2}, polygon, false));

  // Points outside
  EXPECT_FALSE(wombat_core::point_in_polygon({-3, 2}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({-3, 2}, polygon, false));
  EXPECT_FALSE(wombat_core::point_in_polygon({11, 5}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({11, 5}, polygon, false));
  EXPECT_FALSE(wombat_core::point_in_polygon({2, -3}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({2, -3}, polygon, false));
}

TEST(TestGridGeometry, PointInConcavePolygon)
{
  /**
   *  _
   * | |_/\
   * |    |___
   * |        |
   *  ---------
   */

  // Starting from bottom-left corner and proceeding clockwise
  std::vector<wombat_core::grid_coord_t> polygon;
  polygon.push_back({0, 0});
  polygon.push_back({0, 5});
  polygon.push_back({2, 5});
  polygon.push_back({2, 4});
  polygon.push_back({4, 4});
  polygon.push_back({5, 5});
  polygon.push_back({6, 4});
  polygon.push_back({6, 2});
  polygon.push_back({8, 2});
  polygon.push_back({8, 0});

  // Points inside
  EXPECT_TRUE(wombat_core::point_in_polygon({1, 1}, polygon));
  EXPECT_TRUE(wombat_core::point_in_polygon({2, 2}, polygon));
  EXPECT_TRUE(wombat_core::point_in_polygon({3, 3}, polygon));
  EXPECT_TRUE(wombat_core::point_in_polygon({4, 2}, polygon));
  EXPECT_TRUE(wombat_core::point_in_polygon({7, 1}, polygon));

  // Points on border
  EXPECT_TRUE(wombat_core::point_in_polygon({0, 0}, polygon, true));
  EXPECT_TRUE(wombat_core::point_in_polygon({0, 0}, polygon, false));
  EXPECT_TRUE(wombat_core::point_in_polygon({0, 3}, polygon, true));
  EXPECT_TRUE(wombat_core::point_in_polygon({0, 3}, polygon, false));
  EXPECT_TRUE(wombat_core::point_in_polygon({3, 4}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({3, 4}, polygon, false));

  // Points outside
  EXPECT_FALSE(wombat_core::point_in_polygon({0, -5}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({0, -5}, polygon, false));
  EXPECT_FALSE(wombat_core::point_in_polygon({-1, -1}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({-1, -1}, polygon, false));
  EXPECT_FALSE(wombat_core::point_in_polygon({-1, 0}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({-1, 0}, polygon, false));
  EXPECT_FALSE(wombat_core::point_in_polygon({0, 6}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({0, 6}, polygon, false));
  EXPECT_FALSE(wombat_core::point_in_polygon({3, 5}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({3, 5}, polygon, false));
  EXPECT_FALSE(wombat_core::point_in_polygon({4, 5}, polygon, true));
  EXPECT_FALSE(wombat_core::point_in_polygon({4, 5}, polygon, false));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
