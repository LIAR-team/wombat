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
  m_real_time_factor = this->declare_parameter(
    "real_time_factor",
    rclcpp::ParameterValue{1.0}).get<double>();
  m_sim_time_update_period = std::chrono::milliseconds(
    this->declare_parameter(
      "sim_time_update_period_ms",
      rclcpp::ParameterValue{5}).get<int>());
  if (m_real_time_factor <= 0.0) {
    throw std::runtime_error("Invalid real time factor: " + std::to_string(m_real_time_factor));
  }
  m_sim_time_pub = this->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock",
    rclcpp::ClockQoS());

  // Map server setup
  std::string map_yaml_filename = this->declare_parameter(
    "map_yaml_filename",
    rclcpp::ParameterValue{std::string("")}).get<std::string>();
  std::string map_frame_id = this->declare_parameter(
    "map_frame_id",
    rclcpp::ParameterValue{std::string("map")}).get<std::string>();
  std::string map_topic_name = this->declare_parameter(
    "map_topic_name",
    rclcpp::ParameterValue{std::string("map")}).get<std::string>();
  if (!map_yaml_filename.empty()) {
    bool map_setup_success = setup_map_manager(map_yaml_filename, map_frame_id, map_topic_name);
    if (!map_setup_success) {
      throw std::runtime_error("Failed to setup map: " + map_yaml_filename);
    }
  }

  // Robots setup
  std::vector<std::string> robot_names = this->declare_parameter(
    "robots",
    rclcpp::ParameterValue{std::vector<std::string>()}).get<std::vector<std::string>>();
  for (const auto & name : robot_names) {
    bool robot_setup_success = setup_robot(name);
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
      sim_time_loop();
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

void Kennel::sim_time_loop()
{
  const auto start_loop_ts = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    // The current simulated time is computed as the delta time since
    // this function started multiplied for the real time factor
    const auto steady_time_since_start = std::chrono::steady_clock::now() - start_loop_ts;
    const auto steady_time_since_start_nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(steady_time_since_start);
    const int64_t sim_time_nanoseconds =
      static_cast<int64_t>(m_real_time_factor * steady_time_since_start_nanoseconds.count());

    // Publish the simulated time
    auto clock_msg = std::make_unique<rosgraph_msgs::msg::Clock>();
    clock_msg->clock = rclcpp::Time(sim_time_nanoseconds, RCL_ROS_TIME);
    m_sim_time_pub->publish(std::move(clock_msg));

    // Sleep until next iteration
    std::this_thread::sleep_for(m_sim_time_update_period);
  }
}

bool Kennel::setup_robot(const std::string & robot_name)
{
  // A robot must have a name
  // TODO: enforce that names are unique
  if (robot_name.empty()) {
    return false;
  }

  std::vector<std::string> remap_rules;
  remap_rules.push_back("--ros-args");
  remap_rules.push_back("-r");
  remap_rules.push_back("__node:=" + robot_name);

  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("use_sim_time", true));

  auto robot_node_options = rclcpp::NodeOptions()
    .arguments(remap_rules)
    .parameter_overrides(parameters);

  auto robot_node = std::make_shared<RobotSim>(robot_node_options);
  m_robots.push_back(robot_node);

  return true;
}

bool Kennel::setup_map_manager(
  const std::string & map_yaml_filename,
  const std::string & map_frame_id,
  const std::string & map_topic_name)
{
  // The map server requires a filename defining the map
  // TODO: check that the file exists and is valid
  // (or check if the map server is already doing that and producing a clear error)
  if (map_yaml_filename.empty()) {
    return false;
  }

  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("yaml_filename", map_yaml_filename));
  if (!map_frame_id.empty()) {
    parameters.push_back(rclcpp::Parameter("frame_id", map_frame_id));
  }
  if (!map_topic_name.empty()) {
    parameters.push_back(rclcpp::Parameter("topic_name", map_topic_name));
  }
  parameters.push_back(rclcpp::Parameter("use_sim_time", true));
  auto map_options = rclcpp::NodeOptions()
    .parameter_overrides(parameters);

  m_map_server = std::make_shared<nav2_map_server::MapServer>(map_options);
  CallbackReturn ret;
  m_map_server->configure(ret);
  if (ret != CallbackReturn::SUCCESS) {
    assert(0 && "Failed to configure");
  }
  m_map_server->activate(ret);
  if (ret != CallbackReturn::SUCCESS) {
    assert(0 && "Failed to activate");
  }

  return true;
}

std::unique_ptr<Kennel::ThreadWithExecutor>
Kennel::start_executor(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node_base);
  auto thread_and_executor = std::make_unique<ThreadWithExecutor>();
  thread_and_executor->thread = std::make_unique<std::thread>(
    [executor]() {
      executor->spin();
    });
  thread_and_executor->executor = executor;

  return thread_and_executor;
}

}  // namespace kennel
