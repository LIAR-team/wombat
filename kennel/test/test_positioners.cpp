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

static size_t compare_maps(
  const std::vector<int8_t> & reference_map,
  const std::vector<int8_t> & test_map)
{
  size_t count_matched = 0;
  if (test_map.size() != reference_map.size()) {
    return count_matched;
  }

  for (size_t i = 0; i < reference_map.size(); i++) {
    if (reference_map[i] == test_map[i]) {
      count_matched++;
    } else {
      if (test_map[i] != -1) {
        throw std::runtime_error("Bad not matched coord");
      }
    }
  }

  return count_matched;
}

struct positioners_test_data_t
{
  kennel::named_plugin_t positioner_plugin;
  kennel::named_params_t positioner_params {kennel::named_params_t()};
  std::string map_yaml_filename {"walls_map.yaml"};
};

class TestPositionersWithParams
  : public TestKennelSingleRobot, public testing::WithParamInterface<positioners_test_data_t>
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();

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

INSTANTIATE_TEST_SUITE_P(
  TestPositioners,
  TestPositionersWithParams,
  ::testing::Values(
    positioners_test_data_t {
  {kennel::names::POSITIONER_LIDAR_SLAM, "map"}
},
    positioners_test_data_t {
  {kennel::names::POSITIONER_LOCAL_SLAM, "map"},
  {{std::make_pair("radius", rclcpp::ParameterValue(1.5))}}
}
  ),
  [](const ::testing::TestParamInfo<TestPositionersWithParams::ParamType> & my_info)
  {
    std::string positioner_typename = my_info.param.positioner_plugin.type;
    // replace all ':' with '_'
    std::replace(
      positioner_typename.begin(), positioner_typename.end(),
      ':', '_');
    return positioner_typename;
  });

TEST_P(TestPositionersWithParams, DriveAndExploreWallsMap)
{
  auto test_params = GetParam();
  auto kennel_params = kennel::KennelParamsConfig()
    .add_robot()
    .set_robot_pose({1.0, 1.0, 0.0})
    .add_positioner_plugin_to_robot(test_params.positioner_plugin, test_params.positioner_params)
    .add_sensor_plugin_to_robot({kennel::names::SENSOR_BUMPER, "bumper"})
    .set_map_yaml_filename(get_data_path("walls_map.yaml"))
    .get();
  kennel_start(kennel_params);

  auto gt_map = robot->get_occupancy_grid(std::chrono::seconds(10), "/ground_truth_map");
  ASSERT_NE(gt_map, nullptr) << "Failed to get gt occupancy grid";
  auto positioner_map = robot->get_occupancy_grid(std::chrono::seconds(10), "map");
  ASSERT_NE(positioner_map, nullptr) << "Failed to get positioner occupancy grid";

  ASSERT_DOUBLE_EQ(gt_map->info.resolution, positioner_map->info.resolution);
  ASSERT_EQ(gt_map->info.width, positioner_map->info.width);
  ASSERT_EQ(gt_map->info.height, positioner_map->info.height);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.position.x, positioner_map->info.origin.position.x);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.position.y, positioner_map->info.origin.position.y);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.position.z, positioner_map->info.origin.position.z);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.orientation.x, positioner_map->info.origin.orientation.x);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.orientation.y, positioner_map->info.origin.orientation.y);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.orientation.z, positioner_map->info.origin.orientation.z);
  ASSERT_DOUBLE_EQ(gt_map->info.origin.orientation.w, positioner_map->info.origin.orientation.w);
  ASSERT_EQ(gt_map->data.size(), positioner_map->data.size());

  size_t previous_matched = 0;
  size_t count_matched = compare_maps(gt_map->data, positioner_map->data);
  ASSERT_GT(count_matched, previous_matched);

  const double goal_rotation = 2 * wombat_core::PI;
  geometry_msgs::msg::Twist rotate_cmd_vel;
  rotate_cmd_vel.angular.z = 1.0;
  double rotation = robot->rotate_angle(goal_rotation, rotate_cmd_vel, std::chrono::seconds(10));
  EXPECT_GE(std::abs(rotation), goal_rotation);

  positioner_map = robot->get_occupancy_grid(std::chrono::seconds(10), "map");
  ASSERT_NE(positioner_map, nullptr) << "Failed to get positioner occupancy grid";

  previous_matched = count_matched;
  count_matched = compare_maps(gt_map->data, positioner_map->data);
  ASSERT_EQ(gt_map->data.size(), positioner_map->data.size());
  ASSERT_GE(count_matched, previous_matched);

  // publish forward velocity command until we bump
  geometry_msgs::msg::Twist forward_cmd_vel;
  forward_cmd_vel.linear.x = 2.5;
  robot->drive_until_condition(
    forward_cmd_vel,
    [this]() {return is_bumped.load();},
    std::chrono::seconds(10));
  ASSERT_TRUE(is_bumped);

  positioner_map = robot->get_occupancy_grid(std::chrono::seconds(10), "map");
  ASSERT_NE(positioner_map, nullptr) << "Failed to get positioner occupancy grid";

  previous_matched = count_matched;
  count_matched = compare_maps(gt_map->data, positioner_map->data);
  ASSERT_EQ(gt_map->data.size(), positioner_map->data.size());
  ASSERT_GT(count_matched, previous_matched);
}

TEST_P(TestPositionersWithParams, FacingWall)
{
  auto test_params = GetParam();
  auto kennel_params = kennel::KennelParamsConfig()
    .add_robot()
    .set_robot_pose({5.0, 3.0, 0.0})
    .add_positioner_plugin_to_robot(test_params.positioner_plugin, test_params.positioner_params)
    .set_map_yaml_filename(get_data_path("walls_map.yaml"))
    .get();
  kennel_start(kennel_params);

  auto gt_map = robot->get_occupancy_grid(std::chrono::seconds(10), "/ground_truth_map");
  ASSERT_NE(gt_map, nullptr) << "Failed to get gt occupancy grid";
  auto lidar_map = robot->get_occupancy_grid(std::chrono::seconds(10), "map");
  ASSERT_NE(lidar_map, nullptr) << "Failed to get lidar occupancy grid";

  size_t count_matched = compare_maps(gt_map->data, lidar_map->data);
  EXPECT_GT(count_matched, 50);

  size_t count_obstacles = 0;
  for (size_t i = 0; i < lidar_map->data.size(); i++) {
    if (lidar_map->data[i] == 100) {
      count_obstacles++;
    }
  }
  EXPECT_GT(count_obstacles, 50);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
