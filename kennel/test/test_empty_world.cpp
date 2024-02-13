// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "kennel/kennel.hpp"
#include "utils.hpp"

class TestKennelRos : public testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Setup node
    node = std::make_shared<rclcpp::Node>("test_node");
    vel_cmd_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Setup ROS 2 executor
    executor = std::make_unique<rclcpp::experimental::executors::EventsExecutor>();
    executor->add_node(node);
    executor_thread = std::make_unique<std::thread>([this]() {executor->spin();});

    // Setup tf
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setUsingDedicatedThread(true);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

    // Setup Kennel
    kennel = std::make_unique<kennel::Kennel>();
    bool load_success = kennel->configure(get_data_path("empty_world.yaml"));
    ASSERT_TRUE(load_success);
    bool start_success = kennel->start();
    ASSERT_TRUE(start_success);
  }

  void TearDown() override
  {
    bool stop_success = kennel->stop();
    ASSERT_TRUE(stop_success);

    executor->cancel();
    executor_thread->join();

    rclcpp::shutdown();
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
      FAIL();
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
      FAIL();
    }
  }

  std::unique_ptr<kennel::Kennel> kennel;

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_publisher;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<rclcpp::Executor> executor;
  std::unique_ptr<std::thread> executor_thread;
};

TEST_F(TestKennelRos, zero_vel_cmd)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  wait_for_base_tf(robot_pose);

  // Ensure that the robot is at the origin
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);

  // publish vel_cmd
  auto start_time = std::chrono::steady_clock::now();
  while (
    (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(250)) &&
    rclcpp::ok())
  {
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    vel_cmd_publisher->publish(std::move(cmd_msg));
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  get_latest_base_tf(robot_pose);

  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);
}

TEST_F(TestKennelRos, drive_straight)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  wait_for_base_tf(robot_pose);

  // Ensure that the robot is at the origin
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);

  // publish vel_cmd
  auto start_time = std::chrono::steady_clock::now();
  const double x_threshold = 3.0;
  while (
    (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) &&
    rclcpp::ok())
  {
    get_latest_base_tf(robot_pose);
    if (robot_pose.transform.translation.x > x_threshold) {
      break;
    }

    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_msg->linear.x = 5.0;
    vel_cmd_publisher->publish(std::move(cmd_msg));
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  EXPECT_GE(robot_pose.transform.translation.x, x_threshold);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);
}

TEST_F(TestKennelRos, turn_in_place)
{
  geometry_msgs::msg::TransformStamped robot_pose;
  wait_for_base_tf(robot_pose);

  // Ensure that the robot is at the origin
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);

  // publish vel_cmd
  auto start_time = std::chrono::steady_clock::now();
  const double theta_threshold = 3.0;
  while (
    (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) &&
    rclcpp::ok())
  {
    get_latest_base_tf(robot_pose);
    if (tf2::getYaw(robot_pose.transform.rotation) > theta_threshold) {
      break;
    }
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_msg->angular.z = 1.0;
    vel_cmd_publisher->publish(std::move(cmd_msg));

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_GE(tf2::getYaw(robot_pose.transform.rotation), theta_threshold);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
