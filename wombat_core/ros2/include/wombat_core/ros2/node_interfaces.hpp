// Copyright 2024 Soragna Alberto.

#pragma once

#include "rclcpp/rclcpp.hpp"

namespace wombat_core
{

/**
 * @brief This class allows to store ROS 2 node interfaces and
 * implements node-like APIs.
 */
class NodeInterfaces
{
public:
  NodeInterfaces() = default;

  template<typename NodeT>
  explicit NodeInterfaces(NodeT node)
  : node_base(node->get_node_base_interface()),
    node_graph(node->get_node_graph_interface()),
    node_logging(node->get_node_logging_interface()),
    node_timers(node->get_node_timers_interface()),
    node_topics(node->get_node_topics_interface()),
    node_services(node->get_node_services_interface()),
    node_clock(node->get_node_clock_interface()),
    node_parameters(node->get_node_parameters_interface()),
    node_time_source(node->get_node_time_source_interface()),
    node_waitables(node->get_node_waitables_interface())
  {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface()
  {
    return node_base;
  }

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr
  get_node_clock_interface()
  {
    return node_clock;
  }

  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
  get_node_graph_interface()
  {
    return node_graph;
  }

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
  get_node_logging_interface()
  {
    return node_logging;
  }

  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr
  get_node_time_source_interface()
  {
    return node_time_source;
  }

  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
  get_node_timers_interface()
  {
    return node_timers;
  }

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
  get_node_topics_interface()
  {
    return node_topics;
  }

  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr
  get_node_services_interface()
  {
    return node_services;
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
  get_node_parameters_interface()
  {
    return node_parameters;
  }

  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr
  get_node_waitables_interface()
  {
    return node_waitables;
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters;
  rclcpp::node_interfaces::NodeTimeSourceInterface::SharedPtr node_time_source;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables;
};

}  // namespace wombat_core
