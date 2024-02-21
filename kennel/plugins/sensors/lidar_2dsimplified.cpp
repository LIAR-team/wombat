// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kennel/common/sensors/range_projection.hpp"
#include "kennel/common/plugin_interface/sensor_publisher.hpp"
#include "kennel/common/types.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_core/math/transformations.hpp"

#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"


namespace kennel
{

/// Lidar 2D sensor plugin
class Lidar2DSimplified : public SensorPublisher<sensor_msgs::msg::LaserScan>
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
    if (!m_static_map || !m_last_laser_pose ||
      m_last_laser_pose->position.x != laser_pose.position.x ||
      m_last_laser_pose->position.y != laser_pose.position.y ||
      tf2::getYaw(m_last_laser_pose->orientation) != tf2::getYaw(laser_pose.orientation))
    {
      m_last_laser_pose = laser_pose;

      m_scan_msg.ranges = compute_laser_ranges(
        *gt_data.map,
        laser_pose,
        m_num_bins,
        std::make_pair(m_scan_msg.angle_min, m_scan_msg.angle_max),
        std::make_pair(m_scan_msg.range_min, m_scan_msg.range_max));

    using xy = boost::geometry::model::d2::point_xy<double>;
    boost::geometry::model::linestring<xy> line;

      auto laser_yaw = tf2::getYaw(m_last_laser_pose->orientation);
      const double angle_increment = (m_scan_msg.angle_max - m_scan_msg.angle_min) / static_cast<double>(m_num_bins);
      std::vector<float> new_ranges;
      std::vector<std::optional<geometry_msgs::msg::Point>> world_polygon;
      for (size_t i = 0; i < m_scan_msg.ranges.size(); i++) {
        const double this_angle = laser_yaw + m_scan_msg.angle_min + angle_increment * static_cast<double>(i);
        geometry_msgs::msg::Point world_pt;
        world_pt.x = m_last_laser_pose->position.x + std::cos(this_angle) * m_scan_msg.ranges[i];
        world_pt.y = m_last_laser_pose->position.x + std::cos(this_angle) * m_scan_msg.ranges[i];
        //world_polygon.push_back(world_pt);
        boost::geometry::append(line, xy{world_pt.x , world_pt.y});
      }

      double coeff = this->get_parameter("collinear_factor").get<double>();

    // Simplify it, using distance of 0.5 units
    boost::geometry::model::linestring<xy> simplified;
    boost::geometry::simplify(line, simplified, coeff);






      assert(m_scan_msg.ranges.size() >= 3);
      assert(m_scan_msg.ranges.size() == world_polygon.size());
      for (size_t i = 1; i < m_scan_msg.ranges.size() - 1; i++) {

        std::optional<geometry_msgs::msg::Point> prev_pt;
        for (int j = i - 1; j >=0; j--) {
          if (world_polygon[j] != std::nullopt) {
            prev_pt = world_polygon[j];
            break;
          }
        }
        if (!prev_pt) {
          throw std::runtime_error("failed to compute prev pt from " + std::to_string(i));
        }

        auto & this_pt = world_polygon[i];
        auto & next_pt = world_polygon[i + 1];
        // Remove this pt if it's collinear
        if ((std::abs(prev_pt->x - this_pt->x) < coeff && std::abs(this_pt->x - next_pt->x) < coeff) ||
          (std::abs(prev_pt->y - this_pt->y) < coeff && std::abs(this_pt->y - next_pt->y) < coeff))
          {
            this_pt = std::nullopt;
          }
      }
      assert(m_scan_msg.ranges.size() == world_polygon.size());
      for (size_t i = 0; i < m_scan_msg.ranges.size(); i++) {
        if (world_polygon[i] == std::nullopt) {
          m_scan_msg.ranges[i] = 0;
        }
      }
    }

    return std::make_unique<sensor_msgs::msg::LaserScan>(m_scan_msg);
  }

  std::vector<default_parameter_info_t> setup_parameters() override
  {
    std::vector<default_parameter_info_t> params_info;
    default_parameter_info_t info;
    info.descriptor = rcl_interfaces::msg::ParameterDescriptor();

    info.name = "collinear_factor";
    info.value = rclcpp::ParameterValue(0.25);
    params_info.push_back(info);

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
PLUGINLIB_EXPORT_CLASS(kennel::Lidar2DSimplified, kennel::SensorInterface)
