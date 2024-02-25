// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_msgs/msg/bumper.hpp"

#include "kennel/kennel_gtest/kennel_config.hpp"
#include "kennel/kennel_gtest/single_robot_fixture.hpp"
#include "kennel/kennel_gtest/utils.hpp"

class LidarSLAMTest : public TestKennelSingleRobot
{
public:
  void setup_with_map(const std::string & map_yaml_path)
  {
    TestKennelSingleRobot::SetUp();

    auto kennel_params = kennel::KennelParamsConfig()
      .add_robot()
      .set_robot_pose({1.0, 1.0, 0.0})
      .add_lidar_slam_positioner_to_robot()
      .add_bumper_to_robot()
      .set_map_yaml_filename(map_yaml_path)
      .get();
    setup_kennel(kennel_params);

    bumper_subscription = robot->node->create_subscription<wombat_msgs::msg::Bumper>(
      "bumper",
      rclcpp::SensorDataQoS(),
      [this](wombat_msgs::msg::Bumper::ConstSharedPtr msg) {
        if (is_bumped != msg->is_pressed) {
          std::cout << "Bumper is now " << (msg->is_pressed ? "pressed" : "not pressed") << std::endl;
        }
        is_bumped = msg->is_pressed;
      });
  }

  rclcpp::SubscriptionBase::SharedPtr bumper_subscription;
  std::atomic<bool> is_bumped {false};
};

class WallsMapLidarSLAMTest : public LidarSLAMTest
{
public:
  void SetUp() override
  {
    this->setup_with_map(get_data_path("walls_map.yaml"));
  }
};

TEST_F(WallsMapLidarSLAMTest, DriveAndExplore)
{
  auto gt_map = robot->get_occupancy_grid(std::chrono::seconds(10), "/ground_truth_map");
  ASSERT_NE(gt_map, nullptr) << "Failed to get gt occupancy grid";
  auto lidar_map = robot->get_occupancy_grid(std::chrono::seconds(10), "map");
  ASSERT_NE(lidar_map, nullptr) << "Failed to get lidar occupancy grid";

  ASSERT_DOUBLE_EQ(gt_map->info.resolution, lidar_map->info.resolution);
  ASSERT_EQ(gt_map->info.width, lidar_map->info.width);
  ASSERT_EQ(gt_map->info.height, lidar_map->info.height);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.position.x, lidar_map->info.origin.position.x);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.position.y, lidar_map->info.origin.position.y);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.position.z, lidar_map->info.origin.position.z);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.orientation.x, lidar_map->info.origin.orientation.x);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.orientation.y, lidar_map->info.origin.orientation.y);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.orientation.z, lidar_map->info.origin.orientation.z);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.orientation.w, lidar_map->info.origin.orientation.w);
  ASSERT_EQ(gt_map->data.size(), lidar_map->data.size());

  size_t previous_matched = 0;
  size_t count_matched = 0;
  for (size_t i = 0; i < gt_map->data.size(); i++) {
    if (gt_map->data[i] == lidar_map->data[i]) {
      count_matched++;
    } else {
      EXPECT_EQ(lidar_map->data[i], -1);
    }
  }
  ASSERT_GT(count_matched, previous_matched);

  const double goal_rotation = 2 * wombat_core::PI;
  geometry_msgs::msg::Twist rotate_cmd_vel;
  rotate_cmd_vel.angular.z = 1.0;
  double rotation = robot->rotate_angle(goal_rotation, rotate_cmd_vel, std::chrono::seconds(10));
  EXPECT_GE(std::abs(rotation), goal_rotation);

  lidar_map = robot->get_occupancy_grid(std::chrono::seconds(10), "map");
  ASSERT_NE(lidar_map, nullptr) << "Failed to get lidar occupancy grid";

  previous_matched = count_matched;
  count_matched = 0;
  ASSERT_EQ(gt_map->data.size(), lidar_map->data.size());
  for (size_t i = 0; i < gt_map->data.size(); i++) {
    if (gt_map->data[i] == lidar_map->data[i]) {
      count_matched++;
    } else {
      EXPECT_EQ(lidar_map->data[i], -1);
    }
  }
  ASSERT_GT(count_matched, previous_matched);

  // publish forward velocity command until we bump
  geometry_msgs::msg::Twist forward_cmd_vel;
  forward_cmd_vel.linear.x = 2.5;
  robot->drive_until_condition(
    forward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(is_bumped);

  lidar_map = robot->get_occupancy_grid(std::chrono::seconds(10), "map");
  ASSERT_NE(lidar_map, nullptr) << "Failed to get lidar occupancy grid";

  previous_matched = count_matched;
  count_matched = 0;
  ASSERT_EQ(gt_map->data.size(), lidar_map->data.size());
  for (size_t i = 0; i < gt_map->data.size(); i++) {
    if (gt_map->data[i] == lidar_map->data[i]) {
      count_matched++;
    } else {
      EXPECT_EQ(lidar_map->data[i], -1);
    }
  }
  ASSERT_GT(count_matched, previous_matched);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
