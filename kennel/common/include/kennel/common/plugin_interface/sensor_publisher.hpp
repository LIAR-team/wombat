// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/sensor_interface.hpp"
#include "kennel/common/types.hpp"
#include "wombat_core/ros2/time.hpp"

namespace kennel
{

/**
 * @brief Base class to implement ROS 2 sensor plugins.
 * Derived classes are expected to implement the logic to build
 * the sensor data from the ground truth information.
 * This class will expect all the sensor data to be contained in a single
 * ROS 2 message which will be published whenever requested to produce sensor data.
 * @tparam MsgT
 */
template<typename MsgT>
class SensorPublisher : public SensorInterface
{
public:
  /**
   * @brief @sa SensorInterface::initialize_sensor
   * @param parent_node ROS 2 node loading the plugin
   * @param sensor_name name for this plugin
   * @param params_prefix prefix to prepend to parameter declarations
   * @return true if success
   */
  bool initialize_sensor(
    rclcpp::Node * parent_node,
    const std::string & sensor_name,
    const std::string & params_prefix) override
  {
    const bool base_success = this->initialize_plugin(
      sensor_name,
      parent_node,
      params_prefix);
    if (!base_success) {
      RCLCPP_WARN(this->get_logger(), "Failed to do basic setup");
      return false;
    }

    const bool params_success = this->declare_sensor_publisher_params(parent_node);
    if (!params_success) {
      RCLCPP_WARN(this->get_logger(), "Failed to setup sensor publisher parameters");
      return false;
    }

    m_rate_controller = std::make_unique<wombat_core::RateController>(
      std::chrono::milliseconds(get_parameter("period_ms").get<int64_t>()),
      m_clock);

    m_sensor_publisher = parent_node->create_publisher<MsgT>(
      get_parameter("topic_name").get<std::string>(),
      rclcpp::SensorDataQoS());

    const bool post_init_success = this->post_init();
    if (!post_init_success) {
      RCLCPP_WARN(this->get_logger(), "Failed to run post-init routine");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Sensor publisher %s initialized", m_sensor_publisher->get_topic_name());
    return true;
  }

  /**
   * @brief @see SensorInterface::produce_sensor_data
   * This implementation will publish the sensor data on a ROS 2 topic
   * @param gt_data ground truth information that can be used to generate sensor data
   */
  void produce_sensor_data(const localization_data_t & gt_data) override
  {
    if (!m_rate_controller->is_ready()) {
      return;
    }
    auto sensor_msg = make_sensor_ros2_msg(gt_data);
    if (!sensor_msg) {
      return;
    }
    m_sensor_publisher->publish(std::move(sensor_msg));
  }

private:
  /////////
  // TO BE IMPLEMENTED BY DERIVED CLASSES

  /**
   * @brief Build a new ROS 2 message containing sensor data
   * @param gt_data ground truth information that can be used to generate sensor data
   * @return std::unique_ptr<MsgT> the sensor data that will be published
   */
  virtual std::unique_ptr<MsgT> make_sensor_ros2_msg(const localization_data_t & gt_data) = 0;

  /////////

  bool declare_sensor_publisher_params(rclcpp::Node * parent_node)
  {
    std::vector<default_parameter_info_t> params_info;
    default_parameter_info_t info;
    info.descriptor = rcl_interfaces::msg::ParameterDescriptor();

    info.name = "topic_name";
    info.value = rclcpp::ParameterValue(m_plugin_name);
    params_info.push_back(info);

    info.name = "period_ms";
    info.value = rclcpp::ParameterValue(50);
    params_info.push_back(info);

    const bool params_success = this->declare_parameters(
      params_info,
      parent_node);

    return params_success;
  }

  std::unique_ptr<wombat_core::RateController> m_rate_controller;
  std::shared_ptr<rclcpp::Publisher<MsgT>> m_sensor_publisher;
};

}  // namespace kennel
