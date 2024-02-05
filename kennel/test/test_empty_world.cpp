// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <chrono>

#include "rclcpp/rclcpp.hpp"
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
    node = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestKennelRos, drive_straight)
{
  rclcpp::experimental::executors::EventsExecutor executor;
  executor.add_node(node);
  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    });

  auto vel_cmd_publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_buffer->setUsingDedicatedThread(true);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);

  auto kennel = std::make_unique<kennel::Kennel>();
  bool load_success = kennel->configure(get_data_path("empty_world.yaml"));
  ASSERT_TRUE(load_success);
  bool start_success = kennel->start();
  ASSERT_TRUE(start_success);

  geometry_msgs::msg::TransformStamped robot_pose;
  try {
    robot_pose = tf_buffer->lookupTransform(
      "ground_truth", "base_link",
      tf2::TimePointZero,
      std::chrono::seconds(20));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
    FAIL();
  }

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
    auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_msg->linear.x = 5.0;
    vel_cmd_publisher->publish(std::move(cmd_msg));
    try {
      robot_pose = tf_buffer->lookupTransform(
        "ground_truth", "base_link",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
      FAIL();
    }

    if (robot_pose.transform.translation.x > x_threshold) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  EXPECT_GE(robot_pose.transform.translation.x, x_threshold);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.translation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(robot_pose.transform.rotation.w, 1.0);

  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);

  executor.cancel();
  executor_thread.join();
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
