// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <cassert>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel_gtest/robot_interface.hpp"
#include "wombat_core/math/angles.hpp"

namespace kennel
{

RobotInterface::RobotInterface(const std::string & robot_namespace)
{
  // Setup node
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  std::vector<std::string> node_arguments;
  if (!robot_namespace.empty()) {
    node_arguments.push_back("--ros-args");
    node_arguments.push_back("-r");
    node_arguments.push_back("__ns:=/" + robot_namespace);
  }
  node_options.arguments(node_arguments);
  node = std::make_shared<rclcpp::Node>("test_interface", node_options);

  m_logger_timer = node->create_wall_timer(
    std::chrono::milliseconds(250),
    [this]() {
      auto robot_pose = get_latest_base_tf();
      if (!robot_pose) {
        RCLCPP_INFO(node->get_logger(), "Robot gt pose not available");
        return;
      }
      RCLCPP_INFO(
        node->get_logger(),
        "Robot gt pose: %f %f %f",
        robot_pose->transform.translation.x, robot_pose->transform.translation.y,
        tf2::getYaw(robot_pose->transform.rotation));
    });

  // Setup ROS 2 executor
  m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  m_executor->add_node(node);
  m_executor_thread = std::make_unique<std::thread>([this]() {m_executor->spin();});

  // Setup tf
  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  m_tf_buffer->setUsingDedicatedThread(true);
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, node, false);
}

RobotInterface::~RobotInterface()
{
  m_executor->cancel();
  m_executor_thread->join();
}

std::optional<geometry_msgs::msg::TransformStamped>
RobotInterface::get_latest_base_tf(
  std::chrono::milliseconds timeout,
  const std::string & from_frame_id,
  const std::string & to_frame_id)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  try {
    robot_pose = m_tf_buffer->lookupTransform(
      from_frame_id, to_frame_id,
      tf2::TimePointZero,
      timeout);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
    return std::nullopt;
  }

  return robot_pose;
}

void RobotInterface::drive_until_condition(
  const geometry_msgs::msg::Twist & cmd,
  const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout,
  const std::string & topic_name)
{
  auto vel_pub = this->get_publisher<geometry_msgs::msg::Twist>(topic_name);

  auto start_time = std::chrono::steady_clock::now();
  while (
    !predicate() &&
    (std::chrono::steady_clock::now() - start_time < timeout) &&
    rclcpp::ok())
  {
    vel_pub->publish(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

double RobotInterface::rotate_angle(
  double target_angle,
  const geometry_msgs::msg::Twist & rotate_cmd_vel,
  std::chrono::milliseconds timeout,
  const std::string & topic_name)
{
  double accumulated_rotation = 0.0;
  auto last_pose = get_latest_base_tf(std::chrono::seconds(10));
  if (!last_pose) {
    return 0.0;
  }
  drive_until_condition(
    rotate_cmd_vel,
    [this, &target_angle, &last_pose, &accumulated_rotation]() {
      auto robot_pose = get_latest_base_tf();
      if (!robot_pose) {
        RCLCPP_ERROR(node->get_logger(), "Failed to query base tf while rotating");
        return true;
      }
      const double delta_yaw = wombat_core::angles_difference(
        tf2::getYaw(robot_pose->transform.rotation),
        tf2::getYaw(last_pose->transform.rotation));
      last_pose = robot_pose;
      accumulated_rotation += delta_yaw;
      return std::abs(accumulated_rotation) > target_angle;
    },
    timeout,
    topic_name);

  return accumulated_rotation;
}

nav_msgs::msg::OccupancyGrid::ConstSharedPtr
RobotInterface::get_occupancy_grid(
  std::chrono::seconds timeout,
  const std::string & topic)
{
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&map](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      map = msg;
    },
    sub_options);

  auto start_time = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start_time < timeout) && rclcpp::ok()) {
    if (map) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return map;
}

}  // namespace kennel
