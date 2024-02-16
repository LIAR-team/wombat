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

class PluginBase
{
public:
  PluginBase() = default;

  virtual ~PluginBase() = default;

  bool initialize_plugin(
    rclcpp::Node * parent_node,
    const std::string & plugin_name,
    const std::string & params_prefix = "");

protected:
  /////////
  // TO BE IMPLEMENTED BY DERIVED CLASSES

  /**
   * @brief Optional definition of plugin-specific initialization logic.
   * This will be run at the end of the standard initialization, so all other members
   * (e.g. logger, publisher, parameters, etc) are guaranteed to be already available.
   * @return true if the initialization is successful.
   */
  virtual bool post_init();

  /////////

  rclcpp::Logger get_logger();

  /**
   * @brief Retrieve a parameter that was set during via the declare_parameters or
   * setup_parameters method.
   * @param param_name name of the parameter
   * @throws std::out_of_range if the parameter is not found
   * @return value of the parameter
   */
  rclcpp::ParameterValue get_parameter(const std::string & param_name);

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

  rclcpp::Clock::SharedPtr m_clock;
  std::string m_plugin_name;

private:
  /////////
  // TO BE IMPLEMENTED BY DERIVED CLASSES

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

  /////////

  std::string m_params_prefix {""};
  std::unordered_map<std::string, rclcpp::ParameterValue> m_parameters;
  std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_log_interface;
};

}  // namespace kennel
