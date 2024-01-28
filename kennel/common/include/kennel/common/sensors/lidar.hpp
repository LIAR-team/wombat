// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "kennel/common/types.hpp"

namespace kennel
{

std::unique_ptr<sensor_msgs::msg::LaserScan>
make_laser_scan_msg(const LocalizationData & gt_data);

}  // namespace kennel
