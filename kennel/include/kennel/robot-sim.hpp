// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/mobile-base.hpp"

class RobotSim : public rclcpp::Node
{
public:
  explicit RobotSim(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::unique_ptr<MobileBase> m_mobile_base;
};
