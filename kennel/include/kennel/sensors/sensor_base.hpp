// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "kennel/types.hpp"

namespace kennel
{

class SensorBase
{
public:
  SensorBase() = default;

  bool register_sensor(
    rclcpp::Node * parent_node,
    const std::string & sensor_name);

  virtual void produce_sensor_data(const LocalizationData & gt_data) = 0;

protected:
  rclcpp::Logger get_logger();

private:
  virtual bool sensor_setup(rclcpp::Node * parent_node) = 0;

  rclcpp::Clock::SharedPtr m_clock;
  std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_log_interface;
  std::string m_sensor_name;
};

}  // namespace kennel
