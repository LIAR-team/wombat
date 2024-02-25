// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel_gtest/single_robot_fixture.hpp"

void TestKennelSingleRobot::SetUp()
{
  rclcpp::init(0, nullptr);
}

void TestKennelSingleRobot::TearDown()
{
  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);

  rclcpp::shutdown();
}

void TestKennelSingleRobot::setup_kennel(
  const rclcpp::ParameterMap & parameter_map,
  const std::string & robot_namespace)
{
  // Setup Kennel
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  kennel = std::make_unique<kennel::Kennel>(node_options);
  kennel->configure(parameter_map);
  bool start_success = kennel->start();
  ASSERT_TRUE(start_success);

  robot = std::make_shared<kennel::RobotInterface>(robot_namespace);
}
