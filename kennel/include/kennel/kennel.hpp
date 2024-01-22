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
#include "rosgraph_msgs/msg/clock.hpp"

#include "kennel/robot-sim.hpp"

class Kennel : public rclcpp::Node
{
  struct ThreadWithExecutor
  {
    std::unique_ptr<std::thread> thread;
    std::shared_ptr<rclcpp::Executor> executor;
  };

public:
  explicit Kennel(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void run();

private:
  bool setup_robot(const std::string & robot_name);

  bool setup_map_manager(
    const std::string & map_yaml_filename,
    const std::string & map_frame_id,
    const std::string & map_topic_name);

  std::unique_ptr<ThreadWithExecutor>
  start_executor(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base);

  void sim_time_loop();

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_sim_time_pub;
  double m_real_time_factor {0.0};
  std::chrono::milliseconds m_sim_time_update_period {};

  std::shared_ptr<nav2_map_server::MapServer> m_map_server;
  std::vector<std::shared_ptr<RobotSim>> m_robots;
  std::vector<std::unique_ptr<ThreadWithExecutor>> m_executors;
  std::unique_ptr<std::thread> m_sim_time_thread;
};
