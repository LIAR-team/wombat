// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <chrono>
#include <cmath>
#include <memory>

#include "nav2_map_server/map_server.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel.hpp"
#include "wombat_core/ros2/parameters.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kennel
{

Kennel::Kennel(const rclcpp::NodeOptions & options)
: m_node_options(options)
{
  m_kennel_node = std::make_shared<rclcpp::Node>("kennel", m_node_options);

  // Declare parameters
  wombat_core::declare_parameter_if_not_declared(
    m_kennel_node->get_node_parameters_interface(),
    "real_time_factor",
    rclcpp::ParameterValue{5.0});
  wombat_core::declare_parameter_if_not_declared(
    m_kennel_node->get_node_parameters_interface(),
    "sim_time_update_period_ms",
    rclcpp::ParameterValue{2});

  wombat_core::declare_parameter_if_not_declared(
    m_kennel_node->get_node_parameters_interface(),
    "map_yaml_filename",
    rclcpp::ParameterValue{std::string("")});
  wombat_core::declare_parameter_if_not_declared(
    m_kennel_node->get_node_parameters_interface(),
    "map_frame_id",
    rclcpp::ParameterValue{std::string("ground_truth")});
  wombat_core::declare_parameter_if_not_declared(
    m_kennel_node->get_node_parameters_interface(),
    "map_topic_name",
    rclcpp::ParameterValue{std::string("ground_truth_map")});

  wombat_core::declare_parameter_if_not_declared(
    m_kennel_node->get_node_parameters_interface(),
    "robots",
    rclcpp::ParameterValue{std::vector<std::string>()});
}

bool Kennel::configure(const rclcpp::ParameterMap & parameter_map)
{
  // Save the parameter map
  m_params_map = parameter_map;

  // Setup kennel node parameters
  const bool set_params_success = wombat_core::set_parameters_from_map(
    m_params_map,
    m_kennel_node->get_node_base_interface(),
    m_kennel_node->get_node_parameters_interface());
  if (!set_params_success) {
    RCLCPP_WARN(this->get_logger(), "Failed to set kennel parameters from config");
    return false;
  }

  // Simulated time setup
  const auto rtf = m_kennel_node->get_parameter("real_time_factor")
    .get_value<double>();
  const auto sim_time_update_period = m_kennel_node->get_parameter("sim_time_update_period_ms")
    .get_value<int>();
  if (rtf <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Real-Time factor must be a positive definite number: %f", rtf);
    return false;
  }
  m_sim_time_manager = std::make_unique<SimTimeManager>(
    m_kennel_node.get(),
    rtf,
    std::chrono::milliseconds(sim_time_update_period));

  // Map server setup
  const auto map_yaml_filename = m_kennel_node->get_parameter("map_yaml_filename")
    .get_value<std::string>();
  const auto map_frame_id = m_kennel_node->get_parameter("map_frame_id")
    .get_value<std::string>();
  const auto map_topic_name = m_kennel_node->get_parameter("map_topic_name")
    .get_value<std::string>();
  if (!map_yaml_filename.empty()) {
    const bool map_setup_success = setup_map_manager(
      map_yaml_filename,
      map_frame_id,
      map_topic_name,
      m_node_options);
    if (!map_setup_success) {
      RCLCPP_WARN(this->get_logger(), "Failed to setup map: %s", map_yaml_filename.c_str());
      return false;
    }
  }

  // Robots setup
  const auto robot_names = m_kennel_node->get_parameter("robots")
    .get_value<std::vector<std::string>>();
  for (const auto & name : robot_names) {
    const bool robot_setup_success = setup_robot(name, m_node_options, m_params_map);
    if (!robot_setup_success) {
      RCLCPP_WARN(this->get_logger(), "Failed to setup robot: %s", name.c_str());
      return false;
    }
  }

  m_is_configured = true;
  return true;
}

bool Kennel::start()
{
  if (m_is_started) {
    RCLCPP_WARN(this->get_logger(), "Kennel is already started");
    return true;
  }

  if (!m_is_configured) {
    RCLCPP_INFO(this->get_logger(), "Kennel was not configured before running: doing it now");
    bool configure_success = this->configure(m_params_map);
    if (!configure_success) {
      RCLCPP_WARN(this->get_logger(), "Failed to configure");
      return false;
    }
  }

  // Start sim time thread
  m_sim_time_thread = std::make_unique<std::thread>(
    [this]() {
      m_sim_time_manager->run();
    });

  // Start executor for the kennel node
  {
    auto executor = start_executor(std::make_shared<wombat_core::NodeInterfaces>(m_kennel_node));
    m_executors.push_back(std::move(executor));
  }

  // Start robot executor threads
  for (const auto & robot_node : m_robots) {
    auto executor = start_executor(std::make_shared<wombat_core::NodeInterfaces>(robot_node));
    m_executors.push_back(std::move(executor));
  }

  // Start map server executor thread
  if (m_map_server) {
    auto executor = start_executor(std::make_shared<wombat_core::NodeInterfaces>(m_map_server));
    m_executors.push_back(std::move(executor));
  }

  m_is_started = true;
  RCLCPP_INFO(this->get_logger(), "Kennel has been started");
  return m_is_started;
}

bool Kennel::stop()
{
  if (!m_is_started) {
    RCLCPP_WARN(this->get_logger(), "Kennel wasn't started, nothing to stop");
    return true;
  }

  for (auto & exec_with_thread : m_executors) {
    // Cancelling a timer that isn't spinning is a no-op.
    // First wait for the timers to be spinning before issuing the signal.
    // Note that if rclcpp context is shutdown we should stop waiting.
    while (!exec_with_thread->executor->is_spinning() && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    exec_with_thread->executor->cancel();
    exec_with_thread->thread->join();
  }
  m_executors.clear();

  while (!m_sim_time_manager->is_running() && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  m_sim_time_manager->stop();
  m_sim_time_thread->join();
  m_sim_time_thread.reset();

  m_is_configured = false;
  m_is_started = false;

  RCLCPP_INFO(this->get_logger(), "Kennel has been stopped");
  return !m_is_started;
}

bool Kennel::setup_robot(
  const std::string & robot_name,
  const rclcpp::NodeOptions & node_options,
  const rclcpp::ParameterMap & parameter_map)
{
  // A robot must have a name
  // TODO: enforce that names are unique
  if (robot_name.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Empty name passed to setup robot");
    return false;
  }

  auto robot_node_options = node_options;

  auto robot_arguments = robot_node_options.arguments();
  robot_arguments.push_back("--ros-args");
  robot_arguments.push_back("-r");
  robot_arguments.push_back("__ns:=/" + robot_name);

  // HACK! Infer the fully qualified name before creating the node
  const std::string fully_qualified_name = "/" + robot_name + "/robot_sim";
  // Copy parameters from the map to the parameters override options
  const auto node_parameters = rclcpp::parameters_from_map(
    parameter_map,
    fully_qualified_name.c_str());
  for (const auto & param : node_parameters) {
    robot_node_options.append_parameter_override(
      param.get_name(),
      param.get_parameter_value());
  }

  robot_node_options.append_parameter_override("use_sim_time", true);
  robot_node_options.arguments(robot_arguments);

  auto robot_node = std::make_shared<RobotSim>(robot_node_options);

  // Validate that our hack was correctly applied
  if (robot_node->get_fully_qualified_name() != fully_qualified_name) {
    RCLCPP_ERROR(
      this->get_logger(), "Expected fqn doesn't match: %s and %s",
      robot_node->get_fully_qualified_name(),
      fully_qualified_name.c_str());
    return false;
  }

  m_robots.push_back(robot_node);
  return true;
}

bool Kennel::setup_map_manager(
  const std::string & map_yaml_filename,
  const std::string & map_frame_id,
  const std::string & map_topic_name,
  const rclcpp::NodeOptions & node_options)
{
  // The map server requires a filename defining the map
  // TODO: check that the file exists and is valid
  // (or check if the map server is already doing that and producing a clear error)
  if (map_yaml_filename.empty()) {
    return false;
  }

  auto map_options = node_options;
  map_options.append_parameter_override("yaml_filename", map_yaml_filename);
  map_options.append_parameter_override("use_sim_time", true);
  if (!map_frame_id.empty()) {
    map_options.append_parameter_override("frame_id", map_frame_id);
  }
  if (!map_topic_name.empty()) {
    map_options.append_parameter_override("topic_name", map_topic_name);
  }
  map_options.use_intra_process_comms(false);

  m_map_server = std::make_shared<nav2_map_server::MapServer>(map_options);
  CallbackReturn ret {CallbackReturn::FAILURE};
  m_map_server->configure(ret);
  if (ret != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Map server configuration failed");
    return false;
  }
  m_map_server->activate(ret);
  if (ret != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Map server activation failed");
    return false;
  }

  return true;
}

std::unique_ptr<Kennel::node_execution_data_t>
Kennel::start_executor(
  std::shared_ptr<wombat_core::NodeInterfaces> node_interfaces)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_interfaces->get_node_base_interface());
  auto data = std::make_unique<node_execution_data_t>();
  data->thread = std::make_unique<std::thread>(
    [executor]() {
      executor->spin();
    });
  data->executor = executor;
  data->node_interfaces = std::move(node_interfaces);

  return data;
}

rclcpp::Logger Kennel::get_logger()
{
  return m_kennel_node->get_logger();
}

}  // namespace kennel
