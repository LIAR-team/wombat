// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kennel/kennel.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_core/ros2/parameters.hpp"

#include "kennel/kennel_gtest/kennel_config.hpp"
#include "kennel/kennel_gtest/single_robot_fixture.hpp"

class EmptyWorldTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();

    auto kennel_params = kennel::KennelParamsConfig()
      .add_robot()
      .set_map_yaml_filename("")
      .get();
    setup_kennel(kennel_params);
  }
};

TEST_F(EmptyWorldTest, zero_vel_cmd)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  wait_for_base_tf(robot_pose);

  // Ensure that the robot is at the start pose
  EXPECT_NEAR(robot_pose.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);

  geometry_msgs::msg::Twist zero_vel_cmd;
  drive_until_condition(
    zero_vel_cmd,
    [this]() {return false;},
    std::chrono::milliseconds(250));

  get_latest_base_tf(robot_pose);
  EXPECT_NEAR(robot_pose.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);
}

TEST_F(EmptyWorldTest, drive_straight)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  wait_for_base_tf(robot_pose);

  // Ensure that the robot is at the start pose
  EXPECT_NEAR(robot_pose.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);

  // Drive distance
  geometry_msgs::msg::Twist forward_cmd_vel;
  forward_cmd_vel.linear.x = 5.0;
  const double x_threshold = 3.0;
  drive_until_condition(
    forward_cmd_vel,
    [this, &x_threshold]()
    {
      geometry_msgs::msg::TransformStamped pose;
      get_latest_base_tf(pose);
      return pose.transform.translation.x > x_threshold;
    },
    std::chrono::seconds(10));

  get_latest_base_tf(robot_pose);
  EXPECT_GE(robot_pose.transform.translation.x, 0.0);
  EXPECT_NEAR(robot_pose.transform.translation.y, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);
}

TEST_F(EmptyWorldTest, turn_in_place)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  wait_for_base_tf(robot_pose);

  // Ensure that the robot is at the start pose
  EXPECT_NEAR(robot_pose.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);

  const double goal_rotation = wombat_core::PI;
  geometry_msgs::msg::Twist rotate_cmd_vel;
  rotate_cmd_vel.angular.z = 2.0;
  double rotation = rotate_angle(goal_rotation, rotate_cmd_vel, std::chrono::seconds(10));

  get_latest_base_tf(robot_pose);
  EXPECT_NEAR(robot_pose.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_GE(std::abs(rotation), goal_rotation);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
