// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kennel/common/plugin_interface/map_positioner.hpp"
#include "kennel/common/types.hpp"
#include "kennel/common/sensors/range_projection.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/polygon_iterator.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_core/math/transformations.hpp"

#include <iostream>

namespace kennel
{

static wombat_core::grid_coord_t world_pt_to_grid_coord_enforce_bounds(
  const geometry_msgs::msg::Point & world_pt,
  const wombat_core::MapMetaDataAdapter & map_info)
{
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  wombat_core::grid_coord_t::Scalar mx = 0;
  wombat_core::grid_coord_t::Scalar my = 0;

  if (world_pt.x < map_info.origin.position.x) {
    mx = 0;
  } else if (world_pt.x > map_info.resolution * map_info.grid_size.x() + map_info.origin.position.x) {
    mx = map_info.grid_size.x() - 1;
  } else {
    mx = static_cast<int>((world_pt.x - map_info.origin.position.x) / map_info.resolution);
  }

  if (world_pt.y < map_info.origin.position.y) {
    my = 0;
  } else if (world_pt.y > map_info.resolution * map_info.grid_size.y() + map_info.origin.position.y) {
    my = map_info.grid_size.y() - 1;
  } else {
    my = static_cast<int>((world_pt.y - map_info.origin.position.y) / map_info.resolution);
  }

  return wombat_core::grid_coord_t{
    mx, my
  };
}

/**
 * @brief Positioner plugin that maps the local area surrounding
 * the robot.
 */
class LidarSLAMPositioner : public MapPositioner
{
private:
  localization_data_t compute_positioner_data(
    const localization_data_t & gt_data) override
  {
    auto frame_id = this->get_parameter("frame_id").get<std::string>();

    // Pose is simply forwarded with a different frame_id
    m_data.robot_pose = gt_data.robot_pose;
    m_data.robot_pose.header.frame_id = frame_id;

    if (!gt_data.map) {
      return m_data;
    }

    if (!m_map) {
      m_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      m_map->header.frame_id = frame_id;
      m_map->info = gt_data.map->info;
      m_map->data = std::vector<int8_t>(static_cast<size_t>(m_map->info.height) * m_map->info.width, -1);
      RCLCPP_INFO(
        get_logger(),
        "Initialized map with size %d %d and resolution %f",
        static_cast<int>(m_map->info.width),
        static_cast<int>(m_map->info.height),
        m_map->info.resolution);
    }
    auto map_info = wombat_core::MapMetaDataAdapter(m_map->info);
    auto cur_pose = wombat_core::transform_to_pose(gt_data.robot_pose.transform);

    double angle_min = this->get_parameter("angle_min").get<double>();
    double angle_max = this->get_parameter("angle_max").get<double>();
    static constexpr size_t NUM_BINS = 100;
    static constexpr double MIN_DIST = 0.0;
    double max_dist = this->get_parameter("range_max").get<double>();

    auto ranges = compute_laser_ranges(
      *gt_data.map,
      cur_pose,
      NUM_BINS,
      std::make_pair(static_cast<float>(angle_min), static_cast<float>(angle_max)),
      std::make_pair(static_cast<float>(MIN_DIST), static_cast<float>(max_dist)));

    auto laser_yaw = tf2::getYaw(cur_pose.orientation);
    const double angle_increment = (angle_max - angle_min) / static_cast<double>(NUM_BINS);
    using xy = boost::geometry::model::d2::point_xy<double>;
    boost::geometry::model::polygon<xy> poly;
    boost::geometry::append(poly, xy{cur_pose.position.x, cur_pose.position.y});
    for (size_t i = 0; i < ranges.size(); i ++) {
        const double this_angle = laser_yaw + angle_min + angle_increment * static_cast<double>(i);
        geometry_msgs::msg::Point world_pt;
        double x = cur_pose.position.x + std::cos(this_angle) * ranges[i];
        double y = cur_pose.position.y + std::sin(this_angle) * ranges[i];
        boost::geometry::append(poly, xy{x, y});
    }
    boost::geometry::append(poly, xy{cur_pose.position.x, cur_pose.position.y});

    double distance_m = this->get_parameter("simplify_distance").get<double>();
    boost::geometry::model::polygon<xy> simple_poly;
    boost::geometry::simplify(poly, simple_poly, distance_m);
    std::vector<wombat_core::grid_coord_t> grid_polygon;
    for (const auto & world_pt : simple_poly.outer()) {
      geometry_msgs::msg::Point pt;
      pt.x = world_pt.x();
      pt.y = world_pt.y();
      auto grid_coord = world_pt_to_grid_coord_enforce_bounds(pt, map_info);
      grid_polygon.push_back(grid_coord);
    }

    auto polygon_iterator = wombat_core::PolygonIterator(
      map_info,
      grid_polygon);
    for (; !polygon_iterator.is_past_end(); ++polygon_iterator) {
      auto maybe_idx = wombat_core::grid_coord_to_index(*polygon_iterator, map_info);
      if (!maybe_idx) {
        throw std::runtime_error("Failed to convert subgrid coord to index");
      }
      m_map->data[*maybe_idx] = gt_data.map->data[*maybe_idx];
    }

    m_data.map = m_map;

    return m_data;
  }

  std::vector<default_parameter_info_t> setup_parameters() override
  {
    std::vector<default_parameter_info_t> params_info;
    default_parameter_info_t info;
    info.descriptor = rcl_interfaces::msg::ParameterDescriptor();

    info.name = "angle_min";
    info.value = rclcpp::ParameterValue(-wombat_core::PI / 3);
    params_info.push_back(info);

    info.name = "angle_max";
    info.value = rclcpp::ParameterValue(wombat_core::PI / 3);
    params_info.push_back(info);

    info.name = "range_max";
    info.value = rclcpp::ParameterValue(5.0);
    params_info.push_back(info);

    info.name = "simplify_distance";
    info.value = rclcpp::ParameterValue(0.2);
    params_info.push_back(info);

    return params_info;
  }

  wombat_core::grid_index_t m_local_radius_grid {0};
  nav_msgs::msg::OccupancyGrid::SharedPtr m_map;
  localization_data_t m_data;
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::LidarSLAMPositioner, kennel::PositionerInterface)
