// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "kennel/common/range_projection.hpp"
#include "kennel/common/plugin_interface/sensor_publisher.hpp"
#include "kennel/common/pose.hpp"
#include "kennel/common/types.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_core/math/transformations.hpp"

namespace kennel
{

/// Lidar 2D sensor plugin
class Lidar2D : public SensorPublisher<sensor_msgs::msg::LaserScan>
{
private:
  bool post_init() override
  {
    m_num_bins = this->get_parameter("num_bins").get<int64_t>();

    m_scan_msg.header.frame_id = this->get_parameter("frame_id").get<std::string>();
    m_scan_msg.angle_min = this->get_parameter("angle_min").get<double>();
    m_scan_msg.angle_max = this->get_parameter("angle_max").get<double>();
    m_scan_msg.angle_increment = (m_scan_msg.angle_max - m_scan_msg.angle_min) / m_num_bins;
    m_scan_msg.range_min = this->get_parameter("range_min").get<double>();
    m_scan_msg.range_max = this->get_parameter("range_max").get<double>();

    m_static_map = this->get_parameter("static_map").get<bool>();

    return true;
  }

  std::unique_ptr<sensor_msgs::msg::LaserScan>
  make_sensor_ros2_msg(const localization_data_t & gt_data) override
  {
    if (!gt_data.map) {
      return nullptr;
    }

    auto laser_pose = wombat_core::transform_to_pose(gt_data.robot_pose.transform);

    m_scan_msg.header.stamp = gt_data.robot_pose.header.stamp;

    // Compute new ranges only if the map is not static or the robot moved
    if (!m_static_map || !m_last_laser_pose || !pose_2d_equal(*m_last_laser_pose, laser_pose)) {
      m_last_laser_pose = laser_pose;

      auto end_coords = compute_laser_projections(
        *gt_data.map,
        laser_pose,
        m_num_bins,
        std::make_pair(m_scan_msg.angle_min, m_scan_msg.angle_max),
        std::make_pair(m_scan_msg.range_min, m_scan_msg.range_max));

      auto map_info = wombat_core::MapMetaDataAdapter(gt_data.map->info);
      auto maybe_laser_coord = wombat_core::world_pt_to_grid_coord(laser_pose.position, map_info);
      if (!maybe_laser_coord) {
        throw std::runtime_error("Laser pose is outside of grid");
      }
      m_scan_msg.ranges = std::vector<float>(end_coords.size());
      for (size_t i = 0; i < end_coords.size(); i++) {
        const int dx = end_coords[i].x() - maybe_laser_coord->x();
        const int dy = end_coords[i].y() - maybe_laser_coord->y();

        m_scan_msg.ranges[i] = static_cast<float>(
          std::sqrt(dx * dx + dy * dy) * map_info.resolution);
      }
    }

    return std::make_unique<sensor_msgs::msg::LaserScan>(m_scan_msg);
  }

  std::vector<default_parameter_info_t> setup_parameters() override
  {
    std::vector<default_parameter_info_t> params_info;
    default_parameter_info_t info;
    info.descriptor = rcl_interfaces::msg::ParameterDescriptor();

    info.name = "frame_id";
    info.value = rclcpp::ParameterValue("base_link");
    params_info.push_back(info);

    info.name = "static_map";
    info.value = rclcpp::ParameterValue(true);
    params_info.push_back(info);

    info.name = "num_bins";
    info.value = rclcpp::ParameterValue(180);
    params_info.push_back(info);

    info.name = "angle_min";
    info.value = rclcpp::ParameterValue(-wombat_core::PI / 3);
    params_info.push_back(info);

    info.name = "angle_max";
    info.value = rclcpp::ParameterValue(wombat_core::PI / 3);
    params_info.push_back(info);

    info.name = "range_min";
    info.value = rclcpp::ParameterValue(0.0);
    params_info.push_back(info);

    info.name = "range_max";
    info.value = rclcpp::ParameterValue(5.0);
    params_info.push_back(info);

    return params_info;
  }

  sensor_msgs::msg::LaserScan m_scan_msg;
  std::optional<geometry_msgs::msg::Pose> m_last_laser_pose {std::nullopt};

  size_t m_num_bins {0};
  bool m_static_map {false};
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::Lidar2D, kennel::SensorInterface)
