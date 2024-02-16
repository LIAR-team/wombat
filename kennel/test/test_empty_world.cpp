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

#include "single_robot_fixture.hpp"
#include "utils.hpp"

class EmptyWorldTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();
    rclcpp::ParameterMap parameter_map;
    ASSERT_NO_THROW(parameter_map = rclcpp::parameter_map_from_yaml_file(get_data_path("single_robot.yaml")));

    write_parameter_map(
      parameter_map,
      "/kennel",
      "map_yaml_filename",
      rclcpp::ParameterValue(""));

    write_parameter_map(
      parameter_map,
      "/my_robot/robot_sim",
      "mobile_base.ground_truth.map_topic_name",
      rclcpp::ParameterValue(""));

    setup_kennel(parameter_map);
  }
};

static constexpr double START_X = 1.0;
static constexpr double START_Y = 1.0;

TEST_F(EmptyWorldTest, zero_vel_cmd)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  wait_for_base_tf(robot_pose);

  // Ensure that the robot is at the start pose
  EXPECT_NEAR(robot_pose.transform.translation.x, START_X, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, START_Y, 1e-6);
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
  EXPECT_NEAR(robot_pose.transform.translation.x, START_X, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, START_Y, 1e-6);
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
  EXPECT_NEAR(robot_pose.transform.translation.x, START_X, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, START_Y, 1e-6);
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
  EXPECT_GE(robot_pose.transform.translation.x, START_X);
  EXPECT_NEAR(robot_pose.transform.translation.y, START_Y, 1e-6);
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
  EXPECT_NEAR(robot_pose.transform.translation.x, START_X, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, START_Y, 1e-6);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);

  geometry_msgs::msg::Twist rotate_cmd_vel;
  rotate_cmd_vel.angular.z = 2.0;
  const double goal_rotation = wombat_core::PI;
  double accumulated_rotation = 0.0;
  geometry_msgs::msg::TransformStamped last_pose;
  get_latest_base_tf(last_pose);
  drive_until_condition(
    rotate_cmd_vel,
    [this, &goal_rotation, &last_pose, &accumulated_rotation]()
    {
      geometry_msgs::msg::TransformStamped pose;
      get_latest_base_tf(pose);
      const double delta_yaw = wombat_core::angles_difference(
        tf2::getYaw(pose.transform.rotation),
        tf2::getYaw(last_pose.transform.rotation));
      last_pose = pose;
      accumulated_rotation += delta_yaw;
      return std::abs(accumulated_rotation) > goal_rotation;
    },
    std::chrono::seconds(10));

  get_latest_base_tf(robot_pose);
  EXPECT_NEAR(robot_pose.transform.translation.x, START_X, 1e-6);
  EXPECT_NEAR(robot_pose.transform.translation.y, START_Y, 1e-6);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_GE(std::abs(accumulated_rotation), goal_rotation);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
