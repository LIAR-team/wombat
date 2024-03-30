// Copyright 2024 Soragna Alberto.

#pragma once

#include <optional>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/plugin_base.hpp"
#include "kennel/common/types.hpp"

namespace kennel
{

/**
 * @brief Base interface class used to define dynamically-loadable
 * positioner plugins.
 */
class PositionerInterface : public PluginBase
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
  virtual bool initialize_positioner(
    const std::string & plugin_name,
    rclcpp::Node * parent_node,
    const std::string & params_prefix) = 0;

  /**
   * @brief Update the pose estimate using the positioner
   * @param gt_data ground truth data to base the update on
   * @param cmd_vel last velocity command
   * @return updated pose or std::nullopt if couldn't compute it
   */
  virtual std::optional<geometry_msgs::msg::TransformStamped> positioner_update(
    const localization_data_t & gt_data,
    const geometry_msgs::msg::TwistStamped & cmd_vel) = 0;
};

}  // namespace kennel
