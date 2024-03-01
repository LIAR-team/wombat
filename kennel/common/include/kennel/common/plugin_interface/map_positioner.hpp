// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/positioner_interface.hpp"
#include "kennel/common/types.hpp"
#include "wombat_core/ros2/time.hpp"

namespace kennel
{

class MapPositioner : public PositionerInterface
{
public:
  /**
   * @brief Initialization function for the plugins.
   * This will be called by the entity loading the plugins, right after their construction.
   * It's meant to be used as a pseudo-constructor, to circumvent the need for plugin
   * classes to have a constructor with no arguments.
   * @param plugin_name name for this plugin
   * @param parent_node ROS 2 node loading the plugin
   * @param params_prefix prefix to prepend to parameter declarations
   * @return true if initialization was successful
   */
  bool initialize_positioner(
    const std::string & plugin_name,
    rclcpp::Node * parent_node,
    const std::string & params_prefix) override;

  /**
   * @brief Update the pose estimate using the positioner
   * @param gt_data ground truth data to base the update on
   * @param cmd_vel last velocity command
   * @return updated pose or std::nullopt if couldn't compute it
   */
  std::optional<geometry_msgs::msg::TransformStamped> positioner_update(
    const localization_data_t & gt_data,
    const geometry_msgs::msg::TwistStamped & cmd_vel) override;

private:
  /////////
  // TO BE IMPLEMENTED BY DERIVED CLASSES

  /**
   * @brief Compute map and pose estimates for this positioner update.
   * This function must be implemented by derived classes
   * @param gt_data ground truth data to base the update on
   * @return localization_data_t updated positioner data
   */
  virtual localization_data_t compute_positioner_data(
    const localization_data_t & gt_data) = 0;

  /////////

  bool declare_map_positioner_params(rclcpp::Node * parent_node);

  std::unique_ptr<wombat_core::RateController> m_rate_controller;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
};

}  // namespace kennel
