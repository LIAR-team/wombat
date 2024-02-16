// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <optional>
#include <string>

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
   * @param parent_node ROS 2 node loading the plugin
   * @param plugin_name name for this plugin
   * @param params_prefix prefix to prepend to parameter declarations
   * @return true if initialization was successful
   */
  virtual bool initialize_positioner(
    rclcpp::Node * parent_node,
    const std::string & plugin_name,
    const std::string & params_prefix = "") = 0;

  virtual std::optional<geometry_msgs::msg::TransformStamped> positioner_update(
    const localization_data_t & gt_data) = 0;
};

}  // namespace kennel
