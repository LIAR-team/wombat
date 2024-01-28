// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "kennel/common/sensors/lidar.hpp"
#include "kennel/common/sensors/sensor_interface.hpp"
#include "kennel/common/sensors/sensor_ros2_base.hpp"
#include "kennel/common/types.hpp"

namespace kennel
{

class Lidar2D : public SensorRos2Base<sensor_msgs::msg::LaserScan>
{
public:
  Lidar2D() = default;

private:
  std::unique_ptr<sensor_msgs::msg::LaserScan>
  make_sensor_ros2_msg(const LocalizationData & gt_data) override
  {
    return make_laser_scan_msg(gt_data);
  }
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::Lidar2D, kennel::SensorInterface)
