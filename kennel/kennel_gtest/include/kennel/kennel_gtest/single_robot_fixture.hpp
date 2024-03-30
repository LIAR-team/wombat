// Copyright 2024 Soragna Alberto.

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

  void kennel_start(const rclcpp::ParameterMap & parameter_map);

  std::unique_ptr<kennel::Kennel> kennel;

  std::shared_ptr<kennel::RobotInterface> robot;
};
