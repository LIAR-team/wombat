// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "kennel/common/types.hpp"

namespace kennel
{

class SensorInterface
{
public:
  SensorInterface() = default;

  virtual bool initialize_sensor(
    rclcpp::Node * parent_node,
    const std::string & sensor_name) = 0;

  virtual void produce_sensor_data(const LocalizationData & gt_data) = 0;
};

}  // namespace kennel
