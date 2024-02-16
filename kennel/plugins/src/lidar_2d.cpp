// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "kennel/common/sensors/lidar.hpp"
#include "kennel/common/plugin_interface/sensor_ros2_base.hpp"
#include "kennel/common/types.hpp"

namespace kennel
{

/// Lidar 2D sensor plugin
class Lidar2D : public SensorRos2Base<sensor_msgs::msg::LaserScan>
{
public:
  Lidar2D() = default;

private:
  std::unique_ptr<sensor_msgs::msg::LaserScan>
  make_sensor_ros2_msg(const localization_data_t & gt_data) override
  {
    return make_laser_scan_msg(gt_data);
  }
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::Lidar2D, kennel::SensorInterface)
