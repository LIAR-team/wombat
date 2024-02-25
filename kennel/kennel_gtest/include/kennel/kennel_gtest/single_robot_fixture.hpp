// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel.hpp"
#include "kennel/kennel_gtest/robot_interface.hpp"

class TestKennelSingleRobot : public testing::Test
{
public:
  void SetUp() override;

  void TearDown() override;

  void setup_kennel(
    const rclcpp::ParameterMap & parameter_map,
    const std::string & robot_namespace = "my_robot");

  std::unique_ptr<kennel::Kennel> kennel;

  std::shared_ptr<kennel::RobotInterface> robot;
};
