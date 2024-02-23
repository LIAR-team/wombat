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

#include "kennel/kennel_gtest/robot_config.hpp"
#include "kennel/kennel_gtest/single_robot_fixture.hpp"
#include "kennel/kennel_gtest/utils.hpp"

class WallsWorldTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();
    rclcpp::ParameterMap parameter_map;
    ASSERT_NO_THROW(parameter_map = rclcpp::parameter_map_from_yaml_file(get_data_path("single_robot.yaml")));

    bool success = wombat_core::write_parameter_map(
      parameter_map,
      "/kennel",
      "map_yaml_filename",
      rclcpp::ParameterValue(get_data_path("walls_map.yaml")));
    ASSERT_TRUE(success);

    success = kennel::config::add_bumper_to_robot_params(parameter_map);
    ASSERT_TRUE(success);

    setup_kennel(parameter_map);
  }
};

TEST_F(WallsWorldTest, PingPongWalls)
{
  geometry_msgs::msg::TransformStamped start_pose;
  wait_for_base_tf(start_pose);
  ASSERT_FALSE(is_bumped);

  // publish forward velocity command until we bump
  geometry_msgs::msg::Twist forward_cmd_vel;
  forward_cmd_vel.linear.x = 5.0;
  drive_until_condition(
    forward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(2));
  ASSERT_TRUE(is_bumped);

  // publish backward velocity command until we are not bumped
  geometry_msgs::msg::Twist backward_cmd_vel;
  backward_cmd_vel.linear.x = -forward_cmd_vel.linear.x;
  drive_until_condition(
    backward_cmd_vel,
    [this]() {return !is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_FALSE(is_bumped);

  // publish backward velocity command until we are bumped again
  drive_until_condition(
    backward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(is_bumped);

  // Rotate approximately 360 degrees
  const double goal_rotation = 2 * wombat_core::PI;
  double accumulated_rotation = 0.0;
  geometry_msgs::msg::TransformStamped last_pose;
  get_latest_base_tf(last_pose);
  geometry_msgs::msg::Twist rotate_cmd_vel;
  rotate_cmd_vel.angular.z = 3.0;
  drive_until_condition(
    rotate_cmd_vel,
    [this, &goal_rotation, &last_pose, &accumulated_rotation]() {
      geometry_msgs::msg::TransformStamped robot_pose;
      get_latest_base_tf(robot_pose);
      const double delta_yaw = wombat_core::angles_difference(
        tf2::getYaw(robot_pose.transform.rotation),
        tf2::getYaw(last_pose.transform.rotation));
      last_pose = robot_pose;
      accumulated_rotation += delta_yaw;
      return std::abs(accumulated_rotation) > goal_rotation;
    },
    std::chrono::seconds(10));
  // In the current implementation, bumpers are 360 deg around a point
  ASSERT_TRUE(is_bumped);
  ASSERT_GT(std::abs(accumulated_rotation), goal_rotation);

  // publish forward velocity command until we are not bumped
  drive_until_condition(
    forward_cmd_vel,
    [this]() {return !is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_FALSE(is_bumped);

  // publish forward velocity command until we bump
  drive_until_condition(
    forward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(is_bumped);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
