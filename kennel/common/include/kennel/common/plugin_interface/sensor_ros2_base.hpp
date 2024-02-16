// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/sensor_interface.hpp"
#include "kennel/common/types.hpp"

namespace kennel
{

/// Structure containing the fields necessary to declare and initialize a
/// ROS 2 parameter.
struct default_parameter_info_t
{
  std::string name;
  rclcpp::ParameterValue value;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
};

/**
 * @brief Base class to implement ROS 2 sensor plugins.
 * Derived classes are expected to implement the logic to build
 * the sensor data from the ground truth information.
 * This class will expect all the sensor data to be contained in a single
 * ROS 2 message which will be published whenever requested to produce sensor data.
 * @tparam MsgT
 */
template<typename MsgT>
class SensorRos2Base : public SensorInterface
{
public:
  SensorRos2Base() = default;

  ~SensorRos2Base() override = default;

  /**
   * @brief @sa SensorInterface::initialize_sensor
   * @param parent_node ROS 2 node loading the plugin
   * @param sensor_name name for this plugin
   * @return true if success
   */
  bool initialize_sensor(
    rclcpp::Node * parent_node,
    const std::string & sensor_name) override;

  /**
   * @brief @see SensorInterface::produce_sensor_data
   * This implementation will publish the sensor data on a ROS 2 topic
   * @param gt_data ground truth information that can be used to generate sensor data
   */
  void produce_sensor_data(const localization_data_t & gt_data) override;

protected:
  rclcpp::Logger get_logger();

  /**
   * @brief Retrieve a parameter that was set during the setup_parameter method.
   * @param param_name name of the parameter
   * @throws std::out_of_range if the parameter is not found
   * @return value of the parameter
   */
  rclcpp::ParameterValue get_parameter(const std::string & param_name);

  rclcpp::Clock::SharedPtr m_clock;

private:
  /////////
  // TO BE IMPLEMENTED BY DERIVED CLASSES

  /**
   * @brief Build a new ROS 2 message containing sensor data
   * @param gt_data ground truth information that can be used to generate sensor data
   * @return std::unique_ptr<MsgT> the sensor data that will be published
   */
  virtual std::unique_ptr<MsgT> make_sensor_ros2_msg(const localization_data_t & gt_data) = 0;

  /**
   * @brief Optional definition of plugin-specific parameters.
   * Derived classes can override this function to define a set of parameters,
   * by providing their names, a default value and information.
   * After the initialization, true value of the parameters will be accessible via the
   * get_parameter method.
   * @return std::vector<default_parameter_info_t> information about the parameters
   * to define
   */
  virtual std::vector<default_parameter_info_t> setup_parameters();

  /**
   * @brief Optional definition of plugin-specific initialization logic.
   * This will be run at the end of the standard initialization, so all other members
   * (e.g. logger, publisher, parameters, etc) are guaranteed to be already available.
   * @return true if the initialization is successful.
   */
  virtual bool post_init();

  /////////

  /**
   * @brief Private function used to declare a set of ROS 2 parameters given
   * their default information.
   * In case of success, the parameters will then be accessible via the get_parameter
   * method.
   * @param default_parameters_info information about the parameters
   * @param parent_node parent ROS 2 node which will own the parameters
   * @return true if parameters have been successfully declared
   */
  bool declare_parameters(
    const std::vector<default_parameter_info_t> & default_parameters_info,
    rclcpp::Node * parent_node);

  std::unordered_map<std::string, rclcpp::ParameterValue> m_parameters;
  std::shared_ptr<rclcpp::Publisher<MsgT>> m_sensor_publisher;
  std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_log_interface;
  std::string m_sensor_name;
};

}  // namespace kennel

#include "kennel/common/plugin_interface/sensor_ros2_base_impl.hpp"
