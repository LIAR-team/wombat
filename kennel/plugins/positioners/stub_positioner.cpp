// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "kennel/common/plugin_interface/map_positioner.hpp"
#include "kennel/common/types.hpp"

namespace kennel
{

/**
 * @brief Stub positioner plugin.
 * It takes as input ground truth map and pose and republishes them
 * with a different frame id.
 */
class StubPositioner : public MapPositioner
{
private:
  localization_data_t compute_positioner_data(
    const localization_data_t & gt_data) override
  {
    auto frame_id = this->get_parameter("frame_id").get<std::string>();

    // Stub positioner forwards ground truth data with a different frame ID
    localization_data_t slam_data;
    slam_data.robot_pose = gt_data.robot_pose;
    slam_data.robot_pose.header.frame_id = frame_id;
    if (gt_data.map) {
      auto slam_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*(gt_data.map));
      slam_map->header.frame_id = frame_id;
      slam_data.map = slam_map;
    }

    return slam_data;
  }
};

}  // namespace kennel

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kennel::StubPositioner, kennel::PositionerInterface)
