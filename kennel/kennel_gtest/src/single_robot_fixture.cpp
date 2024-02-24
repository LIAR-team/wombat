// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

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

#include "kennel/kennel_gtest/single_robot_fixture.hpp"
#include "kennel/kennel_gtest/utils.hpp"
#include "wombat_core/math/angles.hpp"


void TestKennelSingleRobot::SetUp()
{
  rclcpp::init(0, nullptr);

  // Setup node
  node = std::make_shared<rclcpp::Node>("test_node");

  m_logger_timer = node->create_wall_timer(
    std::chrono::milliseconds(250),
    [this]() {
      geometry_msgs::msg::TransformStamped robot_pose;
      if (m_first_logger_timer_run) {
        this->wait_for_base_tf(robot_pose);
        m_first_logger_timer_run = false;
      }
      get_latest_base_tf(robot_pose);
      RCLCPP_INFO(
        node->get_logger(),
        "Robot pose: %f %f %f",
        robot_pose.transform.translation.x, robot_pose.transform.translation.y,
        tf2::getYaw(robot_pose.transform.rotation));
    });

  // Setup ROS 2 executor
  executor = std::make_unique<rclcpp::experimental::executors::EventsExecutor>();
  executor->add_node(node);
  executor_thread = std::make_unique<std::thread>([this]() {executor->spin();});

  // Setup tf
  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_buffer->setUsingDedicatedThread(true);
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
}

void TestKennelSingleRobot::TearDown()
{
  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);

  executor->cancel();
  executor_thread->join();

  rclcpp::shutdown();
}

void TestKennelSingleRobot::setup_kennel(const rclcpp::ParameterMap & parameter_map)
{
  // Setup Kennel
  kennel = std::make_unique<kennel::Kennel>();
  kennel->configure(parameter_map);
  bool start_success = kennel->start();
  ASSERT_TRUE(start_success);
}

void TestKennelSingleRobot::wait_for_base_tf(
  geometry_msgs::msg::TransformStamped & robot_pose,
  const std::string & from_frame_id,
  const std::string & to_frame_id,
  std::chrono::milliseconds timeout)
{
  try {
    robot_pose = tf_buffer->lookupTransform(
      from_frame_id, to_frame_id,
      tf2::TimePointZero,
      timeout);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
    assert(0 && "Failed to wait for base tf");
  }
}

void TestKennelSingleRobot::get_latest_base_tf(
  geometry_msgs::msg::TransformStamped & robot_pose,
  const std::string & from_frame_id,
  const std::string & to_frame_id)
{
  try {
    robot_pose = tf_buffer->lookupTransform(
      from_frame_id, to_frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
    assert(0 && "Failed to get base tf");
  }
}

void TestKennelSingleRobot::drive_until_condition(
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

double TestKennelSingleRobot::rotate_angle(
  double target_angle,
  const geometry_msgs::msg::Twist & rotate_cmd_vel,
  std::chrono::milliseconds timeout,
  const std::string & topic_name)
{
  double accumulated_rotation = 0.0;
  geometry_msgs::msg::TransformStamped last_pose;
  get_latest_base_tf(last_pose);
  drive_until_condition(
    rotate_cmd_vel,
    [this, &target_angle, &last_pose, &accumulated_rotation]() {
      geometry_msgs::msg::TransformStamped robot_pose;
      get_latest_base_tf(robot_pose);
      const double delta_yaw = wombat_core::angles_difference(
        tf2::getYaw(robot_pose.transform.rotation),
        tf2::getYaw(last_pose.transform.rotation));
      last_pose = robot_pose;
      accumulated_rotation += delta_yaw;
      return std::abs(accumulated_rotation) > target_angle;
    },
    timeout,
    topic_name);

  return accumulated_rotation;
}

nav_msgs::msg::OccupancyGrid::ConstSharedPtr
TestKennelSingleRobot::get_occupancy_grid(
  const std::string & topic,
  std::chrono::seconds timeout)
{
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&map](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      map = msg;
    });

  auto start_time = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start_time < timeout) && rclcpp::ok()) {
    if (map) {return map;}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  assert(0 && "Failed to get occupancy grid");
}
