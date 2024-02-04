// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_map_server/map_server.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/robot_sim.hpp"
#include "kennel/sim_time_manager.hpp"
#include "wombat_core/ros2/node_interfaces.hpp"

namespace kennel
{

/**
 * @brief Main entry-point for the Kennel library.
 * Used to setup one or more simulated robots in the environment.
 */
class Kennel
{
  struct node_execution_data_t
  {
    std::unique_ptr<std::thread> thread;
    std::shared_ptr<rclcpp::Executor> executor;
    std::shared_ptr<wombat_core::NodeInterfaces> node_interfaces;
  };

public:
  /**
   * @brief Construct the kennel.
   * @param options ROS 2 node options that will be forwarded to all internal nodes
   */
  explicit Kennel(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /** @brief Start running the kennel until instructed to stop */
  void run();

  /** @brief Stop the kennel. This function will block until it's fully stopped */
  void stop();

private:
  bool setup_robot(
    const std::string & robot_name,
    const rclcpp::NodeOptions & node_options);

  bool setup_map_manager(
    const std::string & map_yaml_filename,
    const std::string & map_frame_id,
    const std::string & map_topic_name,
    const rclcpp::NodeOptions & node_options);

  rclcpp::Logger get_logger();

  std::unique_ptr<node_execution_data_t>
  start_executor(
    std::shared_ptr<wombat_core::NodeInterfaces> node_interfaces);

  std::unique_ptr<SimTimeManager> m_sim_time_manager;
  std::shared_ptr<rclcpp::Node> m_kennel_node;
  std::shared_ptr<nav2_map_server::MapServer> m_map_server;
  std::vector<std::shared_ptr<RobotSim>> m_robots;
  std::vector<std::unique_ptr<node_execution_data_t>> m_executors;
  std::unique_ptr<std::thread> m_sim_time_thread;
};

}  // namespace kennel
