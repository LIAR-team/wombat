// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "kennel/common/plugin_interface/sensor_publisher.hpp"
#include "kennel/common/types.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/neighbors.hpp"
#include "wombat_msgs/msg/bumper.hpp"

namespace kennel
{

/// Binary bumper sensor plugin
class Bumper : public SensorPublisher<wombat_msgs::msg::Bumper>
{
private:
  std::unique_ptr<wombat_msgs::msg::Bumper>
  make_sensor_ros2_msg(const localization_data_t & gt_data) override
  {
    // TODO: should lookup the frame_id parameter and use a tf buffer
    // to find the location of the sensor.
    // This could also be done during startup, as this is a static transformation
    auto bumper_msg = std::make_unique<wombat_msgs::msg::Bumper>();

    // We can't compute bumper data without a ground truth map
    if (!gt_data.map) {
      return bumper_msg;
    }
    const auto & map = *(gt_data.map);

    geometry_msgs::msg::Point current_pt;
    current_pt.x = gt_data.robot_pose.transform.translation.x;
    current_pt.y = gt_data.robot_pose.transform.translation.y;

    auto current_pose_grid_index = wombat_core::world_pt_to_grid_index(current_pt, map.info);
    if (!current_pose_grid_index) {
      RCLCPP_WARN(this->get_logger(), "Failed to compute grid index for (%f %f)", current_pt.x, current_pt.y);
      return bumper_msg;
    }

    auto is_bumped_func = [&map, &bumper_msg](wombat_core::grid_index_t index) {
        bumper_msg->is_pressed = map.data[index] > 0;
        return bumper_msg->is_pressed;
      };

    // The bumper should have a direction, but we consider it at 360 deg for simplicity purposes
    wombat_core::for_each_grid_neighbor(
      *current_pose_grid_index,
      map.info.width,
      map.info.height,
      is_bumped_func,
      true);

    return bumper_msg;
  }

  std::vector<default_parameter_info_t> setup_parameters() override
  {
    std::vector<default_parameter_info_t> params_info;
    default_parameter_info_t info;
    info.descriptor = rcl_interfaces::msg::ParameterDescriptor();

    info.name = "frame_id";
    info.value = rclcpp::ParameterValue("base_link");
    params_info.push_back(info);

    return params_info;
  }
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::Bumper, kennel::SensorInterface)
