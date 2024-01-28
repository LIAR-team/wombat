// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "kennel/mobile_base/mobile_base.hpp"
#include "kennel/sensors/sensor_interface.hpp"

namespace kennel
{

class RobotSim : public rclcpp::Node
{
public:
  explicit RobotSim(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void sensors_update();

  std::unique_ptr<MobileBase> m_mobile_base;

  std::vector<std::unique_ptr<SensorInterface>> m_sensors;
  rclcpp::TimerBase::SharedPtr m_sensors_timer;
};

}  // namespace kennel
