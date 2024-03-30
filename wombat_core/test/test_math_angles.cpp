// Copyright 2022 Soragna Alberto.

#include <gtest/gtest.h>

#include "wombat_core/math/angles.hpp"

TEST(test_angles, deg_to_rad)
{
  using wombat_core::PI;
  using wombat_core::deg_to_rad;

  constexpr double epsilon = 1e-9;

  EXPECT_NEAR(0.0, deg_to_rad(0.0), epsilon);
  EXPECT_NEAR(PI / 2, deg_to_rad(90.0), epsilon);
  EXPECT_NEAR(PI, deg_to_rad(180.0), epsilon);
  EXPECT_NEAR(PI * 3 / 2, deg_to_rad(270.0), epsilon);
  EXPECT_NEAR(2 * PI, deg_to_rad(360.0), epsilon);
  EXPECT_NEAR(PI / 3, deg_to_rad(60.0), epsilon);
  EXPECT_NEAR(PI * 2 / 3, deg_to_rad(120.0), epsilon);
  EXPECT_NEAR(PI / 4, deg_to_rad(45.0), epsilon);
  EXPECT_NEAR(PI * 3 / 4, deg_to_rad(135.0), epsilon);
  EXPECT_NEAR(PI / 6, deg_to_rad(30.0), epsilon);
}

TEST(test_angles, rad_to_deg)
{
  using wombat_core::PI;
  using wombat_core::rad_to_deg;

  constexpr double epsilon = 1e-9;

  EXPECT_NEAR(rad_to_deg(0.0), 0, epsilon);
  EXPECT_NEAR(rad_to_deg(PI / 2), 90, epsilon);
  EXPECT_NEAR(rad_to_deg(PI), 180, epsilon);
  EXPECT_NEAR(rad_to_deg(PI * 3 / 2), 270, epsilon);
  EXPECT_NEAR(rad_to_deg(2 * PI), 360, epsilon);
  EXPECT_NEAR(rad_to_deg(PI / 3), 60, epsilon);
  EXPECT_NEAR(rad_to_deg(PI * 2 / 3), 120, epsilon);
  EXPECT_NEAR(rad_to_deg(PI / 4), 45, epsilon);
  EXPECT_NEAR(rad_to_deg(PI * 3 / 4), 135, epsilon);
  EXPECT_NEAR(rad_to_deg(PI / 6), 30, epsilon);
}

TEST(test_angles, wrap_angle)
{
  using wombat_core::PI;
  using wombat_core::wrap_angle;

  constexpr double epsilon = 1e-9;

  EXPECT_NEAR(0, wrap_angle(0.0), epsilon);
  EXPECT_NEAR(PI, wrap_angle(PI), epsilon);
  EXPECT_NEAR(0, wrap_angle(2 * PI), epsilon);
  EXPECT_NEAR(PI, wrap_angle(3 * PI), epsilon);
  EXPECT_NEAR(0, wrap_angle(4 * PI), epsilon);

  EXPECT_NEAR(0, wrap_angle(-0.0), epsilon);
  EXPECT_NEAR(-PI, wrap_angle(-PI), epsilon);
  EXPECT_NEAR(0, wrap_angle(-2 * PI), epsilon);
  EXPECT_NEAR(-PI, wrap_angle(-3 * PI), epsilon);
  EXPECT_NEAR(0, wrap_angle(-4 * PI), epsilon);

  EXPECT_NEAR(0, wrap_angle(-0.0), epsilon);
  EXPECT_NEAR(-PI / 2, wrap_angle(-PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, wrap_angle(-3 * PI / 2), epsilon);
  EXPECT_NEAR(0, wrap_angle(-4 * PI / 2), epsilon);

  EXPECT_NEAR(0, wrap_angle(0.0), epsilon);
  EXPECT_NEAR(PI / 2, wrap_angle(PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, wrap_angle(5 * PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, wrap_angle(9 * PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, wrap_angle(-3 * PI / 2), epsilon);
}

TEST(test_angles, angles_difference)
{
  using wombat_core::PI;
  using wombat_core::angles_difference;

  constexpr double epsilon = 1e-9;

  EXPECT_NEAR(-PI / 2, angles_difference(0.0, PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, angles_difference(0.0, -PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, angles_difference(PI / 2, 0.0), epsilon);
  EXPECT_NEAR(-PI / 2, angles_difference(-PI / 2, 0.0), epsilon);

  EXPECT_NEAR(PI / 2, angles_difference(PI, PI / 2), epsilon);
  EXPECT_NEAR(-PI / 2, angles_difference(PI, -PI / 2), epsilon);
  EXPECT_NEAR(-PI / 2, angles_difference(PI / 2, PI), epsilon);
  EXPECT_NEAR(PI / 2, angles_difference(-PI / 2, PI), epsilon);

  EXPECT_NEAR(PI / 2, angles_difference(5 * PI, PI / 2), epsilon);
  EXPECT_NEAR(-PI / 2, angles_difference(7 * PI, -PI / 2), epsilon);
  EXPECT_NEAR(-PI / 2, angles_difference(9 * PI / 2, PI), epsilon);
  EXPECT_NEAR(-PI / 2, angles_difference(-3 * PI / 2, PI), epsilon);

  // Backside wrapping
  EXPECT_NEAR(PI / 2, angles_difference(-3 * PI / 4, 3 * PI / 4), epsilon);
  EXPECT_NEAR(-PI / 2, angles_difference(3 * PI / 4, -3 * PI / 4), epsilon);
}

TEST(test_angles, shortest_angular_distance)
{
  using wombat_core::PI;
  using wombat_core::shortest_angular_distance;

  constexpr double epsilon = 1e-9;

  EXPECT_NEAR(PI / 2, shortest_angular_distance(0.0, PI / 2), epsilon);
  EXPECT_NEAR(-PI / 2, shortest_angular_distance(0.0, -PI / 2), epsilon);
  EXPECT_NEAR(-PI / 2, shortest_angular_distance(PI / 2, 0.0), epsilon);
  EXPECT_NEAR(PI / 2, shortest_angular_distance(-PI / 2, 0.0), epsilon);

  EXPECT_NEAR(-PI / 2, shortest_angular_distance(PI, PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, shortest_angular_distance(PI, -PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, shortest_angular_distance(PI / 2, PI), epsilon);
  EXPECT_NEAR(-PI / 2, shortest_angular_distance(-PI / 2, PI), epsilon);

  EXPECT_NEAR(-PI / 2, shortest_angular_distance(5 * PI, PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, shortest_angular_distance(7 * PI, -PI / 2), epsilon);
  EXPECT_NEAR(PI / 2, shortest_angular_distance(9 * PI / 2, PI), epsilon);
  EXPECT_NEAR(PI / 2, shortest_angular_distance(-3 * PI / 2, PI), epsilon);

  // Backside wrapping
  EXPECT_NEAR(-PI / 2, shortest_angular_distance(-3 * PI / 4, 3 * PI / 4), epsilon);
  EXPECT_NEAR(PI / 2, shortest_angular_distance(3 * PI / 4, -3 * PI / 4), epsilon);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
