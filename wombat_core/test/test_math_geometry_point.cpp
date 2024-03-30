// Copyright 2021-2022 Soragna Alberto.

#include <gtest/gtest.h>

#include "wombat_core/math/geometry_point.hpp"

TEST(test_geometry, points_distance_2d)
{
  geometry_msgs::msg::Point p1;
  p1.x = 10;
  p1.y = 0;
  p1.z = 0;

  EXPECT_EQ(0.0, wombat_core::points_squared_distance_2d(p1, p1));
  EXPECT_EQ(0.0, wombat_core::points_distance_2d(p1, p1));

  geometry_msgs::msg::Point p2;
  p2.x = 10;
  p2.y = 10;
  p2.z = 0;

  EXPECT_EQ(100.0, wombat_core::points_squared_distance_2d(p1, p2));
  EXPECT_EQ(100.0, wombat_core::points_squared_distance_2d(p2, p1));
  EXPECT_EQ(10.0, wombat_core::points_distance_2d(p1, p2));
  EXPECT_EQ(10.0, wombat_core::points_distance_2d(p2, p1));

  geometry_msgs::msg::Point p3;
  p3.x = 10;
  p3.y = 10;
  p3.z = 100;

  EXPECT_EQ(100.0, wombat_core::points_squared_distance_2d(p1, p3));
  EXPECT_EQ(100.0, wombat_core::points_squared_distance_2d(p3, p1));
  EXPECT_EQ(10.0, wombat_core::points_distance_2d(p1, p3));
  EXPECT_EQ(10.0, wombat_core::points_distance_2d(p3, p1));

  geometry_msgs::msg::Point p4;
  p4.x = -10;
  p4.y = 10;
  p4.z = 100;

  EXPECT_EQ(500.0, wombat_core::points_squared_distance_2d(p1, p4));
  EXPECT_EQ(500.0, wombat_core::points_squared_distance_2d(p4, p1));
  EXPECT_NEAR(22.3607, wombat_core::points_distance_2d(p1, p4), 1e-3);
  EXPECT_NEAR(22.3607, wombat_core::points_distance_2d(p4, p1), 1e-3);
}

TEST(test_geometry, points_distance)
{
  geometry_msgs::msg::Point p1;
  p1.x = 10;
  p1.y = 0;
  p1.z = 0;

  EXPECT_EQ(0.0, wombat_core::points_squared_distance(p1, p1));
  EXPECT_EQ(0.0, wombat_core::points_distance(p1, p1));

  geometry_msgs::msg::Point p2;
  p2.x = 10;
  p2.y = 0;
  p2.z = 10;

  EXPECT_EQ(100.0, wombat_core::points_squared_distance(p1, p2));
  EXPECT_EQ(100.0, wombat_core::points_squared_distance(p2, p1));
  EXPECT_EQ(10.0, wombat_core::points_distance(p1, p2));
  EXPECT_EQ(10.0, wombat_core::points_distance(p2, p1));

  geometry_msgs::msg::Point p3;
  p3.x = 10;
  p3.y = -3;
  p3.z = -4;

  EXPECT_EQ(25.0, wombat_core::points_squared_distance(p1, p3));
  EXPECT_EQ(25.0, wombat_core::points_squared_distance(p3, p1));
  EXPECT_EQ(5.0, wombat_core::points_distance(p1, p3));
  EXPECT_EQ(5.0, wombat_core::points_distance(p3, p1));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
