// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <gtest/gtest.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "kennel/kennel.hpp"
#include "wombat_msgs/msg/bumper.hpp"

class TestKennelSingleRobot : public testing::Test
{
public:
  void SetUp() override;

  void TearDown() override;

  void setup_kennel(const rclcpp::ParameterMap & parameter_map);

  void wait_for_base_tf(
    geometry_msgs::msg::TransformStamped & robot_pose,
    const std::string & from_frame_id = "ground_truth",
    const std::string & to_frame_id = "base_link");

  void get_latest_base_tf(
    geometry_msgs::msg::TransformStamped & robot_pose,
    const std::string & from_frame_id = "ground_truth",
    const std::string & to_frame_id = "base_link");

  void drive_until_condition(
    const geometry_msgs::msg::Twist & cmd,
    std::function<bool()> predicate,
    std::chrono::milliseconds timeout = std::chrono::seconds(10),
    const std::string topic_name = "/cmd_vel");

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr
  get_occupancy_grid(
    const std::string topic = "/ground_truth_map",
    std::chrono::seconds timeout = std::chrono::seconds(5));

  std::unique_ptr<kennel::Kennel> kennel;

  std::shared_ptr<rclcpp::Node> node;
  std::atomic<bool> is_bumped {false};
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers;
  rclcpp::Subscription<wombat_msgs::msg::Bumper>::SharedPtr bumper_subscription;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<rclcpp::Executor> executor;
  std::unique_ptr<std::thread> executor_thread;

private:
  rclcpp::TimerBase::SharedPtr m_logger_timer;
};
