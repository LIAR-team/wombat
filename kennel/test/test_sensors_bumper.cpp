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
#include "kennel/kennel_gtest/utils.hpp"

class BumperTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();

    bumper_subscription = node->create_subscription<wombat_msgs::msg::Bumper>(
      "/my_robot/bumper",
      rclcpp::SensorDataQoS(),
      [this](wombat_msgs::msg::Bumper::ConstSharedPtr msg) {
        if (is_bumped != msg->is_pressed) {
          RCLCPP_INFO(
            node->get_logger(),
            "Bumper is now %s",
            (msg->is_pressed ? "pressed" : "not pressed"));
        }
        is_bumped = msg->is_pressed;
      });

    auto kennel_params = kennel::KennelParamsConfig()
      .set_map_yaml_filename(get_data_path("walls_map.yaml"))
      .add_robot()
      .set_robot_pose({1.0, 1.0, 0.0})
      .add_bumper_to_robot()
      .get();

    setup_kennel(kennel_params);
  }

  rclcpp::SubscriptionBase::SharedPtr bumper_subscription;
  std::atomic<bool> is_bumped {false};
};

TEST_F(BumperTest, PingPongWalls)
{
  std::this_thread::sleep_for(std::chrono::seconds(1));
  geometry_msgs::msg::TransformStamped start_pose;
  wait_for_base_tf(start_pose);
  ASSERT_FALSE(is_bumped);

  // publish forward velocity command until we bump
  geometry_msgs::msg::Twist forward_cmd_vel;
  forward_cmd_vel.linear.x = 5.0;
  drive_until_condition(
    forward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(10));
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
  geometry_msgs::msg::Twist rotate_cmd_vel;
  rotate_cmd_vel.angular.z = 3.0;
  double accumulated_rotation = rotate_angle(goal_rotation, rotate_cmd_vel, std::chrono::seconds(10));
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
