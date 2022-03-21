// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include <gtest/gtest.h>

#include <iostream>

#include "wombat_control/data_types/pose2d.hpp"
#include "wombat_control/models/model_f_euler_dis.hpp"

TEST(test_model, zero_vels_integration)
{
  double delta_t = 0.1;  // [s]
  Pose2D initial_pose = {0, 0, 0};
  KinModel unicycle = KinModel(initial_pose);
  Pose2D output_pose = unicycle.integration(0, 0, delta_t);
  EXPECT_EQ(output_pose, initial_pose);
}

TEST(test_model, omega_integration)
{
  double delta_t = 0.1;  // [s]
  Pose2D initial_pose = {0, 0, 0};
  KinModel unicycle = KinModel(initial_pose);
  Pose2D output_pose = unicycle.integration(0, 1, delta_t);
  Pose2D expected_pose = {0, 0, delta_t};
  EXPECT_EQ(output_pose, expected_pose);
}

TEST(test_model, vel_integration)
{
  double delta_t = 0.1;  // [s]

  Pose2D initial_pose = {0, 0, 0};
  KinModel unicycle = KinModel(initial_pose);
  Pose2D output_pose = unicycle.integration(1, 0, delta_t);
  Pose2D expected_pose = {delta_t, 0, 0};
  EXPECT_EQ(output_pose, expected_pose);
}

TEST(test_model, multiple_integrations)
{
  double delta_t = 0.1;  // [s]
  double final_time = 2;  // [s]

  Pose2D initial_pose = {0, 0, 0};
  Pose2D output_pose;
  Pose2D expected_pose;
  KinModel unicycle = KinModel(initial_pose);
  for (double i = 0; i <= final_time; i = i + delta_t) {
    output_pose = unicycle.integration(1, 0, delta_t);
    expected_pose = {i + delta_t, 0, 0};
    EXPECT_EQ(output_pose, expected_pose);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
