// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace wombat_core
{

class RateController
{
public:
  RateController(
    double frequency_hz,
    rclcpp::Clock::SharedPtr clock,
    bool start_ready = true);

  RateController(
    std::chrono::nanoseconds period,
    rclcpp::Clock::SharedPtr clock,
    bool start_ready = true);

  bool is_ready();

  bool sleep_until_ready();

private:
  rclcpp::Duration m_period {std::chrono::nanoseconds(0)};
  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Time m_last_ready_time;
};

}  // namespace wombat_core
