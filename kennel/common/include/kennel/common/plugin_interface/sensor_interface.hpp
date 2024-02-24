// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/plugin_base.hpp"
#include "kennel/common/types.hpp"

namespace kennel
{

/**
 * @brief Base interface class used to define dynamically-loadable
 * sensor plugins.
 */
class SensorInterface : public PluginBase
{
public:
  /**
   * @brief Initialization function for the plugins.
   * This will be called by the entity loading the plugins, right after their construction.
   * It's meant to be used as a pseudo-constructor, to circumvent the need for plugin
   * classes to have a constructor with no arguments.
   * @param parent_node ROS 2 node loading the plugin
   * @param sensor_name name for this plugin
   * @param params_prefix prefix to prepend to parameter declarations
   * @return true if initialization was successful
   */
  virtual bool initialize_sensor(
    rclcpp::Node * parent_node,
    const std::string & sensor_name,
    const std::string & params_prefix) = 0;

  /**
   * @brief This function will be periodically called to generate sensor data.
   * Derived classes should use the ground truth information to build a simulated
   * sensor data and produce it.
   * @param gt_data current ground truth information
   */
  virtual void produce_sensor_data(const localization_data_t & gt_data) = 0;
};

}  // namespace kennel
