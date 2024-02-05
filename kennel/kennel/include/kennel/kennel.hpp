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
 * Used to setup and run one or more simulated robots in the environment.
 * The use of this class is not thread-safe.
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
  void start();

  /** @brief Stop the kennel. This function will block until it's fully stopped */
  void stop();

  /**
   * @brief Configure the Kennel, preparing it for the next start
   * @param yaml_file_path optional path to a yaml configuration file
   * @return true if configuration is successful
   */
  bool configure(const std::string & yaml_file_path = "");

private:
  /**
   * @brief Loads a yaml file and sets parameters for the provided node
   * @param yaml_file_path path to the yaml file to read
   * @param node_base_ifc node base interface for the node where to load parameters
   * @param node_parameters_ifc node parameters interface for the node where to load parameters
   * @return true if file was loaded and parameters set successfully
   */
  bool load_parameters_from_yaml(
    const std::string & yaml_file_path,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_ifc,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_ifc);

  bool setup_robot(
    const std::string & robot_name,
    const rclcpp::NodeOptions & node_options,
    const std::string & yaml_file_path);

  bool setup_map_manager(
    const std::string & map_yaml_filename,
    const std::string & map_frame_id,
    const std::string & map_topic_name,
    const rclcpp::NodeOptions & node_options);

  rclcpp::Logger get_logger();

  std::unique_ptr<node_execution_data_t>
  start_executor(
    std::shared_ptr<wombat_core::NodeInterfaces> node_interfaces);

  rclcpp::NodeOptions m_node_options;
  bool m_is_configured {false};
  bool m_is_started {false};

  std::unique_ptr<SimTimeManager> m_sim_time_manager;
  std::shared_ptr<rclcpp::Node> m_kennel_node;
  std::shared_ptr<nav2_map_server::MapServer> m_map_server;
  std::vector<std::shared_ptr<RobotSim>> m_robots;
  std::vector<std::unique_ptr<node_execution_data_t>> m_executors;
  std::unique_ptr<std::thread> m_sim_time_thread;
};

}  // namespace kennel
