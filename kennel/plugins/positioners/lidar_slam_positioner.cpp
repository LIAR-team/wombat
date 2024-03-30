// Copyright 2024 Soragna Alberto.

#include <memory>

#include "boost/geometry.hpp"
#include "boost/geometry/algorithms/append.hpp"
#include "boost/geometry/algorithms/simplify.hpp"
#include "boost/geometry/geometries/polygon.hpp"
#include "boost/geometry/geometries/point_xy.hpp"

#include "kennel/common/plugin_interface/map_positioner.hpp"
#include "kennel/common/pose.hpp"
#include "kennel/common/types.hpp"
#include "kennel/common/range_projection.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/polygon_iterator.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_core/math/transformations.hpp"

namespace kennel
{

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
    auto static_map = this->get_parameter("static_map").get<bool>();

    // Pose is simply forwarded with a different frame_id
    m_data.robot_pose = gt_data.robot_pose;
    m_data.robot_pose.header.frame_id = frame_id;

    if (!gt_data.map) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *m_clock, 1000,
        "ground truth data map not available");
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
    m_map->header.stamp = gt_data.map->header.stamp;

    // Compute new map only if the map is not static or the robot moved
    auto cur_pose = wombat_core::transform_to_pose(gt_data.robot_pose.transform);
    if (static_map && m_last_robot_pose && pose_2d_equal(*m_last_robot_pose, cur_pose)) {
      return m_data;
    }
    m_last_robot_pose = cur_pose;

    this->update_map(gt_data, cur_pose);

    m_data.map = m_map;
    return m_data;
  }

  void update_map(
    const localization_data_t & gt_data,
    const geometry_msgs::msg::Pose & cur_pose)
  {
    auto map_info = wombat_core::MapMetaDataAdapter(m_map->info);

    double angle_min = this->get_parameter("angle_min").get<double>();
    double angle_max = this->get_parameter("angle_max").get<double>();
    static constexpr size_t NUM_BINS = 100;
    static constexpr double MIN_DIST = 0.0;
    double max_dist = this->get_parameter("range_max").get<double>();

    auto laser_end_coords = compute_laser_projections(
      *gt_data.map,
      cur_pose,
      NUM_BINS,
      std::make_pair(static_cast<float>(angle_min), static_cast<float>(angle_max)),
      std::make_pair(static_cast<float>(MIN_DIST), static_cast<float>(max_dist)));

    const auto maybe_robot_coord = wombat_core::world_pt_to_grid_coord(
      cur_pose.position,
      map_info);
    if (!maybe_robot_coord) {
      throw std::runtime_error("Robot is outside the map");
    }

    using boost_xy = boost::geometry::model::d2::point_xy<int>;
    boost::geometry::model::polygon<boost_xy> poly;
    boost::geometry::append(poly, boost_xy{maybe_robot_coord->x(), maybe_robot_coord->y()});
    for (const auto & coord : laser_end_coords) {
      boost::geometry::append(poly, boost_xy{coord.x(), coord.y()});
    }
    boost::geometry::append(poly, boost_xy{maybe_robot_coord->x(), maybe_robot_coord->y()});

    double distance_grid =
      this->get_parameter("simplify_distance").get<double>() / map_info.resolution;
    boost::geometry::model::polygon<boost_xy> simple_poly;
    boost::geometry::simplify(poly, simple_poly, distance_grid);
    // TODO: we should ensure that after simplification we still have a valid polygon
    if (!simple_poly.outer().empty()) {
      std::vector<wombat_core::grid_coord_t> grid_polygon;
      for (const auto & boost_coord : simple_poly.outer()) {
        grid_polygon.emplace_back(boost_coord.x(), boost_coord.y());
      }

      // We need to include the boundaries in the polygon iterator, otherwise we may
      // end up not rendering the walls in the map.
      auto polygon_iterator = wombat_core::PolygonIterator(
        map_info,
        grid_polygon,
        true);
      for (; !polygon_iterator.is_past_end(); ++polygon_iterator) {
        auto maybe_idx = wombat_core::grid_coord_to_index(*polygon_iterator, map_info);
        if (!maybe_idx) {
          throw std::runtime_error("Failed to convert subgrid coord to index");
        }
        m_map->data[*maybe_idx] = gt_data.map->data[*maybe_idx];
      }
    }
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
    info.value = rclcpp::ParameterValue(2.5);
    params_info.push_back(info);

    info.name = "simplify_distance";
    info.value = rclcpp::ParameterValue(0.2);
    params_info.push_back(info);

    info.name = "static_map";
    info.value = rclcpp::ParameterValue(true);
    params_info.push_back(info);

    return params_info;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr m_map;
  localization_data_t m_data;
  std::optional<geometry_msgs::msg::Pose> m_last_robot_pose;
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::LidarSLAMPositioner, kennel::PositionerInterface)
