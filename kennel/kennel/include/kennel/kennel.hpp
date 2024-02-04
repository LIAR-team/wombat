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

namespace kennel
{

/**
 * @brief Main entry-point for the Kennel library.
 * Used to setup one or more simulated robots in the environment.
 */
class Kennel : public rclcpp::Node
{
  struct thread_with_executor_t
  {
    std::unique_ptr<std::thread> thread;
    std::shared_ptr<rclcpp::Executor> executor;
  };

public:
  explicit Kennel(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void run();

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

  std::unique_ptr<thread_with_executor_t>
  start_executor(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base);

  std::unique_ptr<SimTimeManager> m_sim_time_manager;
  std::shared_ptr<nav2_map_server::MapServer> m_map_server;
  std::vector<std::shared_ptr<RobotSim>> m_robots;
  std::vector<std::unique_ptr<thread_with_executor_t>> m_executors;
  std::unique_ptr<std::thread> m_sim_time_thread;
};

}  // namespace kennel
