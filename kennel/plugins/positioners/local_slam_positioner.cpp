// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "kennel/common/plugin_interface/map_positioner.hpp"
#include "kennel/common/types.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/submap_iterator.hpp"
#include "wombat_core/math/transformations.hpp"

namespace kennel
{

/**
 * @brief Positioner plugin that maps the local area surrounding
 * the robot.
 */
class LocalSLAMPositioner : public MapPositioner
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
      RCLCPP_INFO(get_logger(), "returning without map");
      return m_data;
    }

    if (!m_map) {
      m_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      m_map->header.frame_id = frame_id;
      m_map->info = gt_data.map->info;
      m_map->data = std::vector<int8_t>(m_map->info.height * m_map->info.width, -1);
      m_local_radius_grid =
        static_cast<wombat_core::grid_index_t>(this->get_parameter("radius").get<double>() / m_map->info.resolution);
      RCLCPP_INFO(
        get_logger(),
        "Initialized map with size %d %d and resolution %f. Subgrid radius %d",
        static_cast<int>(m_map->info.width),
        static_cast<int>(m_map->info.height),
        m_map->info.resolution,
        static_cast<int>(m_local_radius_grid));
    }

    auto cur_pose = wombat_core::transform_to_pose(gt_data.robot_pose.transform);
    auto maybe_cur_grid_coord = wombat_core::world_pt_to_grid_coord(cur_pose.position, m_map->info);
    if (!maybe_cur_grid_coord) {
      assert(0 && "bar cur pos");
    }

    auto top_left_corner = wombat_core::grid_coord_bounded_diff(
      *maybe_cur_grid_coord,
      wombat_core::grid_coord_t{m_local_radius_grid, m_local_radius_grid});
    auto bottom_right_corner = wombat_core::grid_coord_bounded_sum(
      *maybe_cur_grid_coord,
      wombat_core::grid_coord_t{m_local_radius_grid, m_local_radius_grid},
      m_map->info);

    auto submap_size = wombat_core::get_subgrid_size_from_corners(
      top_left_corner,
      bottom_right_corner);

    /*
    RCLCPP_INFO(this->get_logger(), "ROBOT_POSE: %d %d TL: %d %d BR %d %d size %d %d",
      static_cast<int>(maybe_cur_grid_coord->x), static_cast<int>(maybe_cur_grid_coord->y),
      static_cast<int>(top_left_corner.x), static_cast<int>(top_left_corner.y),
      static_cast<int>(bottom_right_corner.x), static_cast<int>(bottom_right_corner.y),
      static_cast<int>(submap_size.x), static_cast<int>(submap_size.y));
    */

    auto submap_iterator = wombat_core::SubmapIterator(
      m_map->info,
      top_left_corner,
      submap_size);
    for (;!submap_iterator.isPastEnd(); ++submap_iterator)
    {
      //RCLCPP_INFO(this->get_logger(), "Got coord: %d %d", static_cast<int>(submap_iterator->x), static_cast<int>(submap_iterator->y));

      auto maybe_idx = wombat_core::grid_coord_to_index(*submap_iterator, m_map->info);
      if (!maybe_idx) {
        assert(0 && "bad submap idx");
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

    info.name = "radius";
    info.value = rclcpp::ParameterValue(0.5);
    params_info.push_back(info);

    return params_info;
  }

  wombat_core::grid_index_t m_local_radius_grid {0};
  nav_msgs::msg::OccupancyGrid::SharedPtr m_map;
  localization_data_t m_data;
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::LocalSLAMPositioner, kennel::PositionerInterface)
