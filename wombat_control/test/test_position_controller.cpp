// Copyright 2021-2022 Azzollini Ilario, Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "wombat_control/models/diff_drive_model.hpp"
#include "wombat_control/control/position_controller.hpp"

inline bool operator==(
  const geometry_msgs::msg::Twist & twist1,
  const geometry_msgs::msg::Twist & twist2)
{
  return
    (twist1.linear.x == twist2.linear.x) &&
    (twist1.linear.y == twist2.linear.y) &&
    (twist1.linear.z == twist2.linear.z) &&
    (twist1.angular.x == twist2.angular.x) &&
    (twist1.angular.y == twist2.angular.y) &&
    (twist1.angular.z == twist2.angular.z);
}

TEST(test_controller, waypoint_control)
{
  auto delta_time = rclcpp::Duration(std::chrono::milliseconds(10));
  double final_time = 15;  // [s]
  double len = 0.5;  // [m]
  double gain_x = 1;
  double gain_y = 1;
  geometry_msgs::msg::Pose robot_pose;
  double goal_x = 10;
  double goal_y = 10;
  bool goal_reached = false;

  // Test 1
  geometry_msgs::msg::Pose current_pose;
  PosCtrl pos_controller = PosCtrl({len, gain_x, gain_y});
  geometry_msgs::msg::Twist input_signal = pos_controller.input_function({0, 0}, current_pose);
  geometry_msgs::msg::Twist expected_input_signal;
  EXPECT_EQ(input_signal, expected_input_signal);

  // Test 2
  double e_x = robot_pose.position.x - goal_x;
  double e_y = robot_pose.position.y - goal_y;
  double t = 0;

  for (t = 0; t <= final_time; t = t + delta_time.seconds()) {
    e_x = robot_pose.position.x - goal_x;
    e_y = robot_pose.position.y - goal_y;
    if (std::abs(e_x) <= 1e-3 && std::abs(e_y) <= 1e-3) {
      goal_reached = true;
      break;
    }
    input_signal = pos_controller.input_function({e_x, e_y}, robot_pose);
    robot_pose = wombat_control::diff_drive_model_integration(
      robot_pose,
      input_signal,
      delta_time);
  }

  EXPECT_TRUE(goal_reached);
}
