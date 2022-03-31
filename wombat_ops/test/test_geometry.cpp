// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include <gtest/gtest.h>

#include "wombat_ops/geometry/point.hpp"

TEST(test_geometry, points_distance)
{
  geometry_msgs::msg::Point p1;
  p1.x = 10;
  p1.y = 0;

  EXPECT_EQ(0.0, wombat_ops::points_squared_distance(p1, p1));
  EXPECT_EQ(0.0, wombat_ops::points_distance(p1, p1));

  geometry_msgs::msg::Point p2;
  p2.x = 10;
  p2.y = 10;

  EXPECT_EQ(100.0, wombat_ops::points_squared_distance(p1, p2));
  EXPECT_EQ(100.0, wombat_ops::points_squared_distance(p2, p1));
  EXPECT_EQ(10.0, wombat_ops::points_distance(p1, p2));
  EXPECT_EQ(10.0, wombat_ops::points_distance(p2, p1));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
