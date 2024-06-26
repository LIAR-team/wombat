// Copyright 2024 Soragna Alberto.

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
#include "wombat_msgs/msg/bumper.hpp"

#include "kennel/kennel_gtest/kennel_config.hpp"
#include "kennel/kennel_gtest/single_robot_fixture.hpp"
#include "kennel/kennel_gtest/utils.hpp"

class BumperTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();

    auto kennel_params = kennel::KennelParamsConfig()
      .set_map_yaml_filename(get_data_path("walls_map.yaml"))
      .add_robot()
      .set_robot_pose({1.0, 1.0, 0.0})
      .add_sensor_plugin_to_robot({kennel::names::SENSOR_BUMPER, "bumper"})
      .get();
    kennel_start(kennel_params);

    bumper_subscription = robot->node->create_subscription<wombat_msgs::msg::Bumper>(
      "/my_robot/bumper",
      rclcpp::SensorDataQoS(),
      [this](wombat_msgs::msg::Bumper::ConstSharedPtr msg) {
        if (is_bumped != msg->is_pressed) {
          std::cout << "Bumper is now " << (msg->is_pressed ? "pressed" : "not pressed") << std::endl;
        }
        is_bumped = msg->is_pressed;
      });
  }

  rclcpp::SubscriptionBase::SharedPtr bumper_subscription;
  std::atomic<bool> is_bumped {false};
};

TEST_F(BumperTest, PingPongWalls)
{
  auto start_pose = robot->get_latest_base_tf(std::chrono::seconds(10));
  ASSERT_NE(start_pose, std::nullopt);
  ASSERT_FALSE(is_bumped);

  // publish forward velocity command until we bump
  geometry_msgs::msg::Twist forward_cmd_vel;
  forward_cmd_vel.linear.x = 5.0;
  robot->drive_until_condition(
    forward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(is_bumped);

  // publish backward velocity command until we are not bumped
  geometry_msgs::msg::Twist backward_cmd_vel;
  backward_cmd_vel.linear.x = -forward_cmd_vel.linear.x;
  robot->drive_until_condition(
    backward_cmd_vel,
    [this]() {return !is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_FALSE(is_bumped);

  // publish backward velocity command until we are bumped again
  robot->drive_until_condition(
    backward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(is_bumped);

  // Rotate approximately 360 degrees
  const double goal_rotation = 2 * wombat_core::PI;
  geometry_msgs::msg::Twist rotate_cmd_vel;
  rotate_cmd_vel.angular.z = 3.0;
  double accumulated_rotation = robot->rotate_angle(goal_rotation, rotate_cmd_vel, std::chrono::seconds(10));
  ASSERT_TRUE(is_bumped);
  ASSERT_GT(std::abs(accumulated_rotation), goal_rotation);

  // publish forward velocity command until we are not bumped
  robot->drive_until_condition(
    forward_cmd_vel,
    [this]() {return !is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_FALSE(is_bumped);

  // publish forward velocity command until we bump
  robot->drive_until_condition(
    forward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(is_bumped);
}

class InternalBumperTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();

    auto kennel_params = kennel::KennelParamsConfig()
      .set_map_yaml_filename(get_data_path("walls_map.yaml"))
      .add_robot()
      .set_robot_pose({1.0, 1.0, 0.0})
      .set_robot_radius(0.15)
      .add_sensor_plugin_to_robot({kennel::names::SENSOR_BUMPER, "bumper"})
      .get();
    kennel_start(kennel_params);

    bumper_subscription = robot->node->create_subscription<wombat_msgs::msg::Bumper>(
      "bumper",
      rclcpp::SensorDataQoS(),
      [this](wombat_msgs::msg::Bumper::ConstSharedPtr msg) {
        if (is_bumped != msg->is_pressed) {
          std::cout << "Bumper is now " << (msg->is_pressed ? "pressed" : "not pressed") << std::endl;
        }
        is_bumped = msg->is_pressed;
      });
  }

  rclcpp::SubscriptionBase::SharedPtr bumper_subscription;
  std::atomic<bool> is_bumped {false};
};

TEST_F(InternalBumperTest, PingPongWalls)
{
  auto start_pose = robot->get_latest_base_tf(std::chrono::seconds(10));
  ASSERT_NE(start_pose, std::nullopt);
  ASSERT_FALSE(is_bumped);

  auto last_pose = start_pose;
  // publish forward velocity command until pose stops updating
  geometry_msgs::msg::Twist forward_cmd_vel;
  forward_cmd_vel.linear.x = 2.5;
  auto last_pose_change_time = robot->node->now();
  auto not_moving_threshold = rclcpp::Duration(std::chrono::milliseconds(500));
  robot->drive_until_condition(
    forward_cmd_vel,
    [this, &last_pose, &last_pose_change_time, &not_moving_threshold]()
    {
      auto cur_pose = robot->get_latest_base_tf();
      if (!cur_pose) {
        throw std::runtime_error("Failed to get latest tf");
      }
      auto now = robot->node->now();
      if (cur_pose->transform.translation.x != last_pose->transform.translation.x) {
        last_pose = cur_pose;
        last_pose_change_time = now;
      }
      return now - last_pose_change_time > not_moving_threshold;
    },
    std::chrono::seconds(10));

  ASSERT_FALSE(is_bumped);

  auto cur_pose = robot->get_latest_base_tf();
  ASSERT_NE(cur_pose, std::nullopt);
  double dx = cur_pose->transform.translation.x - start_pose->transform.translation.x;
  EXPECT_GT(dx, 0.1);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
