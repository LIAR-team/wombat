// Copyright 2021-2022 Azzollini Ilario, Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "wombat_control/models/diff_drive_model.hpp"
#include "wombat_core/math/angles.hpp"

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
  geometry_msgs::msg::Pose initial_pose;
  EXPECT_NEAR(initial_pose.position.x, 0.0, 1e-5);
  EXPECT_NEAR(initial_pose.orientation.w, 1.0, 1e-5);

  geometry_msgs::msg::Twist command;
  geometry_msgs::msg::Pose output_pose = wombat_control::diff_drive_model_integration(
    initial_pose,
    command,
    rclcpp::Duration(std::chrono::milliseconds(100))
  );
  EXPECT_EQ(output_pose, initial_pose);
}

TEST(test_model, omega_integration)
{
  auto delta_time = rclcpp::Duration(std::chrono::milliseconds(100));
  geometry_msgs::msg::Twist command;
  command.angular.z = 1.0;
  auto output_pose = wombat_control::diff_drive_model_integration(
    geometry_msgs::msg::Pose(),
    command,
    delta_time);
  geometry_msgs::msg::Pose expected_pose;
  expected_pose.orientation = wombat_core::quaternion_from_rpy(0.0, 0.0, delta_time.seconds());

  EXPECT_EQ(output_pose, expected_pose);
}

TEST(test_model, vel_integration)
{
  auto delta_time = rclcpp::Duration(std::chrono::milliseconds(100));

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  auto output_pose = wombat_control::diff_drive_model_integration(
    geometry_msgs::msg::Pose(),
    command,
    delta_time);
  geometry_msgs::msg::Pose expected_pose;
  expected_pose.position.x = delta_time.seconds();
  EXPECT_EQ(output_pose, expected_pose);
}

TEST(test_model, multiple_integrations)
{
  auto delta_time = rclcpp::Duration(std::chrono::milliseconds(100));
  double final_time = 2.0;  // [s]

  geometry_msgs::msg::Pose robot_pose;
  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  for (double i = 0; i <= final_time; i = i + delta_time.seconds()) {
    auto output_pose = wombat_control::diff_drive_model_integration(
      robot_pose,
      command,
      delta_time);
    geometry_msgs::msg::Pose expected_pose;
    expected_pose.position.x = i + delta_time.seconds();
    EXPECT_EQ(output_pose, expected_pose);
    robot_pose = output_pose;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
