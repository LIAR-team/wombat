// Copyright 2021-2022 Azzollini Ilario, Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "wombat_control/models/diff_drive_model.hpp"

inline bool operator==(
  const geometry_msgs::msg::Pose & pose1,
  const geometry_msgs::msg::Pose & pose2)
{
  return
    (pose1.position.x == pose2.position.x) &&
    (pose1.position.y == pose2.position.y) &&
    (pose1.position.z == pose2.position.z) &&
    (pose1.orientation.x == pose2.orientation.x) &&
    (pose1.orientation.y == pose2.orientation.y) &&
    (pose1.orientation.z == pose2.orientation.z) &&
    (pose1.orientation.w == pose2.orientation.w);
}

TEST(test_model, zero_vels_integration)
{
  double dt = 0.1;  // [s]
  geometry_msgs::msg::Pose initial_pose;
  EXPECT_NEAR(initial_pose.position.x, 0.0, 1e-5);
  EXPECT_NEAR(initial_pose.orientation.w, 1.0, 1e-5);

  DiffDriveModel unicycle = DiffDriveModel(initial_pose);
  geometry_msgs::msg::Twist command;
  geometry_msgs::msg::Pose output_pose = unicycle.integration(command, dt);
  EXPECT_EQ(output_pose, initial_pose);
}

TEST(test_model, omega_integration)
{
  double dt = 0.1;  // [s]
  DiffDriveModel unicycle = DiffDriveModel(geometry_msgs::msg::Pose());
  geometry_msgs::msg::Twist command;
  command.angular.z = 1.0;
  geometry_msgs::msg::Pose output_pose = unicycle.integration(command, dt);
  geometry_msgs::msg::Pose expected_pose;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, dt);
  expected_pose.orientation = tf2::toMsg(q);

  EXPECT_EQ(output_pose, expected_pose);
}

TEST(test_model, vel_integration)
{
  double dt = 0.1;  // [s]
  DiffDriveModel unicycle = DiffDriveModel(geometry_msgs::msg::Pose());

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  geometry_msgs::msg::Pose output_pose = unicycle.integration(command, dt);
  geometry_msgs::msg::Pose expected_pose;
  expected_pose.position.x = dt;
  EXPECT_EQ(output_pose, expected_pose);
}

TEST(test_model, multiple_integrations)
{
  double dt = 0.1;  // [s]
  double final_time = 2.0;  // [s]
  DiffDriveModel unicycle = DiffDriveModel(geometry_msgs::msg::Pose());

  geometry_msgs::msg::Pose output_pose;
  for (double i = 0; i <= final_time; i = i + dt) {
    geometry_msgs::msg::Twist command;
    command.linear.x = 1.0;
    output_pose = unicycle.integration(command, dt);
    geometry_msgs::msg::Pose expected_pose;
    expected_pose.position.x = i + dt;
    EXPECT_EQ(output_pose, expected_pose);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
