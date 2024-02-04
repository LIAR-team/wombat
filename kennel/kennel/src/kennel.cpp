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

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kennel
{

Kennel::Kennel(const rclcpp::NodeOptions & options)
: rclcpp::Node("kennel", options)
{
  // Simulated time setup
  const auto rtf = this->declare_parameter(
    "real_time_factor",
    rclcpp::ParameterValue{1.0}).get<double>();
  const auto sim_time_update_period = std::chrono::milliseconds(
    this->declare_parameter(
      "sim_time_update_period_ms",
      rclcpp::ParameterValue{5}).get<int>());
  if (rtf <= 0.0) {
    throw std::runtime_error("Invalid real time factor: " + std::to_string(rtf));
  }
  m_sim_time_manager = std::make_unique<SimTimeManager>(
    this,
    rtf,
    std::chrono::milliseconds(sim_time_update_period));

  // Map server setup
  const std::string map_yaml_filename = this->declare_parameter(
    "map_yaml_filename",
    rclcpp::ParameterValue{std::string("")}).get<std::string>();
  const std::string map_frame_id = this->declare_parameter(
    "map_frame_id",
    rclcpp::ParameterValue{std::string("map")}).get<std::string>();
  const std::string map_topic_name = this->declare_parameter(
    "map_topic_name",
    rclcpp::ParameterValue{std::string("map")}).get<std::string>();
  if (!map_yaml_filename.empty()) {
    const bool map_setup_success = setup_map_manager(
      map_yaml_filename,
      map_frame_id,
      map_topic_name,
      options);
    if (!map_setup_success) {
      throw std::runtime_error("Failed to setup map: " + map_yaml_filename);
    }
  }

  // Robots setup
  const std::vector<std::string> robot_names = this->declare_parameter(
    "robots",
    rclcpp::ParameterValue{std::vector<std::string>()}).get<std::vector<std::string>>();
  for (const auto & name : robot_names) {
    const bool robot_setup_success = setup_robot(name, options);
    if (!robot_setup_success) {
      throw std::runtime_error("Failed to setup robot: " + name);
    }
  }
}

void Kennel::run()
{
  // Start sim time thread
  m_sim_time_thread = std::make_unique<std::thread>(
    [this]() {
      m_sim_time_manager->run();
    });

  // Start executor for this node
  {
    auto executor = start_executor(this->get_node_base_interface());
    m_executors.push_back(std::move(executor));
  }

  // Start robot executor threads
  for (const auto & robot_node : m_robots) {
    auto executor = start_executor(robot_node->get_node_base_interface());
    m_executors.push_back(std::move(executor));
  }

  // Start map server executor thread
  if (m_map_server) {
    auto executor = start_executor(m_map_server->get_node_base_interface());
    m_executors.push_back(std::move(executor));
  }
}

void Kennel::stop()
{
  for (auto & exec_with_thread : m_executors) {
    exec_with_thread->executor->cancel();
    exec_with_thread->thread->join();
  }
  m_executors.clear();

  m_sim_time_manager->stop();
  m_sim_time_thread->join();
  m_sim_time_thread.reset();
}

bool Kennel::setup_robot(
  const std::string & robot_name,
  const rclcpp::NodeOptions & node_options)
{
  // A robot must have a name
  // TODO: enforce that names are unique
  if (robot_name.empty()) {
    return false;
  }

  auto robot_node_options = node_options;

  auto robot_arguments = robot_node_options.arguments();
  robot_arguments.push_back("--ros-args");
  robot_arguments.push_back("-r");
  robot_arguments.push_back("__ns:=/" + robot_name);

  robot_node_options.append_parameter_override("use_sim_time", true);
  robot_node_options.arguments(robot_arguments);

  auto robot_node = std::make_shared<RobotSim>(robot_node_options);
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

std::unique_ptr<Kennel::thread_with_executor_t>
Kennel::start_executor(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_base);
  auto thread_and_executor = std::make_unique<thread_with_executor_t>();
  thread_and_executor->thread = std::make_unique<std::thread>(
    [executor]() {
      executor->spin();
    });
  thread_and_executor->executor = executor;

  return thread_and_executor;
}

}  // namespace kennel
