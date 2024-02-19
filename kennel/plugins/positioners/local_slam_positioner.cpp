// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "kennel/common/plugin_interface/map_positioner.hpp"
#include "kennel/common/types.hpp"
#include "wombat_core/math/grid/coordinates.hpp"
#include "wombat_core/math/grid/SubmapIterator.hpp"
#include "wombat_core/math/transformations.hpp"

#include <iostream>

namespace kennel
{

static void boundIndexToRange(int & index, const int & bufferSize)
{
  if (index < 0) {
    index = 0;
  } else if (index >= bufferSize) {
    index = bufferSize - 1;
  }
}

static void boundIndexToRange(
  wombat_core::Index & index,
  const wombat_core::Size & bufferSize)
{
  for (int i = 0; i < index.size(); i++) {
    boundIndexToRange(index[i], bufferSize[i]);
  }
}

static wombat_core::Size getSubmapSizeFromCornerIndices(
  const wombat_core::Index & topLeftIndex,
  const wombat_core::Index & bottomRightIndex)
{
  return wombat_core::Size(bottomRightIndex - topLeftIndex + wombat_core::Size::Ones());
}

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

    // Stub positioner forwards ground truth data with a different frame ID
    m_data.robot_pose = gt_data.robot_pose;
    m_data.robot_pose.header.frame_id = frame_id;

    if (!gt_data.map) {
      RCLCPP_INFO(get_logger(), "returning without map");
      return m_data;
    }

    if (!m_map) {
      RCLCPP_INFO(get_logger(), "Creating new map");
      m_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      m_map->header.frame_id = frame_id;
      m_map->info = gt_data.map->info;
      m_map->data = std::vector<int8_t>(m_map->info.height * m_map->info.width, -1);
      RCLCPP_INFO(get_logger(), "Created new map DONE");
    }

    auto cur_pose = wombat_core::transform_to_pose(gt_data.robot_pose.transform);
    auto maybe_cur_grid_coord = wombat_core::world_pt_to_grid_coord(cur_pose.position, m_map->info);
    if (!maybe_cur_grid_coord) {
      assert(0 && "bar cur pos");
    }
    RCLCPP_INFO(get_logger(), "Cur pose computed");

    size_t width = 100;
    size_t height = 100;

    wombat_core::Index this_index = {maybe_cur_grid_coord->x, maybe_cur_grid_coord->y};
    RCLCPP_INFO(get_logger(), "This index %d %d", this_index.x(), this_index.y());
    wombat_core::Index other_index = {this_index.x() + width, this_index.y() + height};
    RCLCPP_INFO(get_logger(), "Other index %d %d", other_index.x(), other_index.y());
    wombat_core::Size map_size = {m_map->info.width, m_map->info.height};
    boundIndexToRange(other_index, map_size);
    RCLCPP_INFO(get_logger(), "Bounded other index %d %d", other_index.x(), other_index.y());

    auto submap_size = getSubmapSizeFromCornerIndices(
      this_index,
      other_index);

    RCLCPP_INFO(get_logger(), "Found new size %d %d", submap_size.x(), submap_size.y());

    //size_t count = 0;
    auto submap_iterator = wombat_core::SubmapIterator(
      m_map->info,
      wombat_core::grid_coord_t{static_cast<unsigned int>(this_index.x()), static_cast<unsigned int>(this_index.y())},
      submap_size.x(),
      submap_size.y());
    for (;!submap_iterator.isPastEnd(); ++submap_iterator)
    {
      //RCLCPP_INFO(get_logger(), "IT: %d", static_cast<int>(count));
      //RCLCPP_INFO(get_logger(), "COORD: %d %d ", (*submap_iterator).x(), (*submap_iterator).y());
      //count++;
      wombat_core::grid_coord_t coord = {static_cast<unsigned int>((*submap_iterator).x()), static_cast<unsigned int>((*submap_iterator).y())};
      auto maybe_idx = wombat_core::grid_coord_to_index(coord, m_map->info);
      if (!maybe_idx) {
        assert(0 && "bad submap idx");
      }

      m_map->data[*maybe_idx] = gt_data.map->data[*maybe_idx];
      //std::cout<<"C: " << ( << " " << (*submap_iterator).y() << std::endl;
    }
    //RCLCPP_INFO(get_logger(), "processed %d cells", static_cast<int>(count));

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

  nav_msgs::msg::OccupancyGrid::SharedPtr m_map;
  localization_data_t m_data;
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::LocalSLAMPositioner, kennel::PositionerInterface)
