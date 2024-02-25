// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace kennel
{

class RobotInterface
{
public:
  explicit RobotInterface(const std::string & robot_namespace);

  virtual ~RobotInterface();

  std::optional<geometry_msgs::msg::TransformStamped> get_latest_base_tf(
    std::chrono::milliseconds timeout = std::chrono::seconds(0),
    const std::string & from_frame_id = "ground_truth",
    const std::string & to_frame_id = "base_link");

  void drive_until_condition(
    const geometry_msgs::msg::Twist & cmd,
    const std::function<bool()> & predicate,
    std::chrono::milliseconds timeout = std::chrono::seconds(10),
    const std::string & topic_name = "cmd_vel");

  double rotate_angle(
    double target_angle,
    const geometry_msgs::msg::Twist & rotate_cmd_vel,
    std::chrono::milliseconds timeout = std::chrono::seconds(10),
    const std::string & topic_name = "cmd_vel");

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr
  get_occupancy_grid(
    std::chrono::seconds timeout = std::chrono::seconds(10),
    const std::string & topic = "map");

  std::shared_ptr<rclcpp::Node> node;

protected:
  template<typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr get_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & publisher_qos = rclcpp::SensorDataQoS())
  {
    auto pub_base_it = m_publishers.find(topic_name);
    if (pub_base_it == m_publishers.end()) {
      auto new_pub = node->create_publisher<MsgT>(topic_name, publisher_qos);
      auto insert_result = m_publishers.insert(std::make_pair(topic_name, new_pub));
      if (!insert_result.second) {
        throw std::runtime_error("Failed to insert new publisher for " + topic_name);
      }
      pub_base_it = insert_result.first;
    }

    auto pub_base = pub_base_it->second;
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<MsgT>>(pub_base);
    if (!pub) {
      throw std::runtime_error("Failed to dynamic cast publisher " + topic_name);
    }

    return pub;
  }

  rclcpp::TimerBase::SharedPtr m_logger_timer;
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> m_publishers;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<rclcpp::Executor> m_executor;
  std::unique_ptr<std::thread> m_executor_thread;
};

}  // namespace kennel
