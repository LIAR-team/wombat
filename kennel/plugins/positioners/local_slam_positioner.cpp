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
      return m_data;
    }

    if (!m_map) {
      m_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      m_map->header.frame_id = frame_id;
      m_map->info = gt_data.map->info;
      m_map->data = std::vector<int8_t>(static_cast<size_t>(m_map->info.height) * m_map->info.width, -1);
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
    auto map_info = wombat_core::MapMetaDataAdapter(m_map->info);

    auto cur_pose = wombat_core::transform_to_pose(gt_data.robot_pose.transform);
    auto maybe_cur_grid_coord = wombat_core::world_pt_to_grid_coord(cur_pose.position, map_info);
    if (!maybe_cur_grid_coord) {
      throw std::runtime_error("Failed to convert world pose to grid coord");
    }
    auto radius_coord = wombat_core::grid_coord_t{m_local_radius_grid, m_local_radius_grid};

    wombat_core::grid_coord_t min_corner = *maybe_cur_grid_coord - radius_coord;
    min_corner = wombat_core::enfouce_bounds_on_grid_coord(min_corner, map_info);
    wombat_core::grid_coord_t max_corner = *maybe_cur_grid_coord + radius_coord;
    max_corner = wombat_core::enfouce_bounds_on_grid_coord(max_corner, map_info);

    auto submap_size = wombat_core::get_grid_size_from_corners(
      min_corner,
      max_corner);

    auto submap_iterator = wombat_core::SubmapIterator(
      map_info,
      min_corner,
      submap_size);
    for (; !submap_iterator.is_past_end(); ++submap_iterator) {
      auto maybe_idx = wombat_core::grid_coord_to_index(*submap_iterator, map_info);
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
