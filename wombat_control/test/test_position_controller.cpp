// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include <assert.h>
#include <gtest/gtest.h>

#include <iostream>

#include "wombat_control/data_types/pose2d.hpp"
#include "wombat_control/model/model_f_euler_dis.hpp"
#include "wombat_control/control/position_controller.hpp"
#include "wombat_control/data_types/vel_commands.hpp"

TEST(test_controller, waypoint_control)
{
  double delta_t = 0.01;  // [s]
  double final_time = 15;  // [s]
  double len = 0.5;  // [m]
  double gain_x = 1;
  double gain_y = 1;
  Pose2D initial_pose = {0, 0, 0};
  double goal_x = 10;
  double goal_y = 10;
  bool goal_reached = false;

  // Test 1
  Pose2D current_pose = {0, 0, 0};
  PosCtrl pos_controller = PosCtrl(len, gain_x, gain_y);
  VelCommands input_signal = pos_controller.input_function(0, 0, current_pose);
  std::cout << input_signal << std::endl;
  VelCommands expected_input_signal = {0, 0};
  assert(input_signal == expected_input_signal);

  // Test 2
  KinModel unicycle = KinModel(initial_pose);
  Pose2D robot_pose = initial_pose;
  double e_x = robot_pose.x - goal_x;
  double e_y = robot_pose.y - goal_y;
  double t = 0;

  for (t = 0; t <= final_time; t = t + delta_t) {
    e_x = robot_pose.x - goal_x;
    e_y = robot_pose.y - goal_y;
    if (std::abs(e_x) <= 1e-3 && std::abs(e_y) <= 1e-3) {
      goal_reached = true;
      break;
    }
    input_signal = pos_controller.input_function(e_x, e_y, robot_pose);
    robot_pose = unicycle.integration(input_signal.v, input_signal.omega, delta_t);
  }

  EXPECT_TRUE(goal_reached);
}
