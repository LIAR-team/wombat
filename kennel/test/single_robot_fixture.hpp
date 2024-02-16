// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "kennel/kennel.hpp"
#include "wombat_msgs/msg/bumper.hpp"
#include "utils.hpp"

class TestKennelSingleRobot : public testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Setup node
    node = std::make_shared<rclcpp::Node>("test_node");
    vel_cmd_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    bumper_subscription = node->create_subscription<wombat_msgs::msg::Bumper>(
      "/my_robot/bumper",
      rclcpp::SensorDataQoS(),
      [this](wombat_msgs::msg::Bumper::ConstSharedPtr msg) {
        is_bumped = msg->is_pressed;
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

  void TearDown() override
  {
    bool stop_success = kennel->stop();
    ASSERT_TRUE(stop_success);

    executor->cancel();
    executor_thread->join();

    rclcpp::shutdown();
  }

  void setup_kennel(const rclcpp::ParameterMap & parameter_map)
  {
    // Setup Kennel
    kennel = std::make_unique<kennel::Kennel>();
    kennel->configure(parameter_map);
    bool start_success = kennel->start();
    ASSERT_TRUE(start_success);
  }

  void wait_for_base_tf(geometry_msgs::msg::TransformStamped & robot_pose)
  {
    try {
      robot_pose = tf_buffer->lookupTransform(
        "ground_truth", "base_link",
        tf2::TimePointZero,
        std::chrono::seconds(20));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
      assert(0 && "Failed to wait for base tf");
    }
  }

  void get_latest_base_tf(geometry_msgs::msg::TransformStamped & robot_pose)
  {
    try {
      robot_pose = tf_buffer->lookupTransform(
        "ground_truth", "base_link",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
      assert(0 && "Failed to get base tf");
    }
  }

  void drive_until_condition(
    const geometry_msgs::msg::Twist & cmd,
    std::function<bool()> predicate,
    std::chrono::milliseconds timeout = std::chrono::seconds(10))
  {
    auto start_time = std::chrono::steady_clock::now();
    while (
      !predicate() &&
      (std::chrono::steady_clock::now() - start_time < timeout) &&
      rclcpp::ok())
    {
      vel_cmd_publisher->publish(cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::unique_ptr<kennel::Kennel> kennel;

  std::shared_ptr<rclcpp::Node> node;
  std::atomic<bool> is_bumped {false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_publisher;
  rclcpp::Subscription<wombat_msgs::msg::Bumper>::SharedPtr bumper_subscription;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<rclcpp::Executor> executor;
  std::unique_ptr<std::thread> executor_thread;
};
