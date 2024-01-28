// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "kennel/common/sensors/lidar.hpp"
#include "kennel/common/types.hpp"
#include "kennel/plugins/lidar_2d.hpp"

namespace kennel
{

std::unique_ptr<sensor_msgs::msg::LaserScan>
Lidar2D::make_sensor_ros2_msg(const LocalizationData & gt_data)
{
  return make_laser_scan_msg(gt_data);
}

}  // namespace kennel
