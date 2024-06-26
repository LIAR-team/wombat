// Copyright 2024 Soragna Alberto.

#include <gtest/gtest.h>

#include <memory>

#include "kennel/common/collisions.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/ros2/parameters.hpp"

#include "kennel/kennel_gtest/kennel_config.hpp"
#include "kennel/kennel_gtest/single_robot_fixture.hpp"
#include "kennel/kennel_gtest/utils.hpp"

class WallsWorldTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();

    auto kennel_params = kennel::KennelParamsConfig()
      .set_map_yaml_filename(get_data_path("walls_map.yaml"))
      .get();

    kennel_start(kennel_params);
  }
};

TEST_F(WallsWorldTest, WallCollisionBis)
{
  auto map = robot->get_occupancy_grid(std::chrono::seconds(10), "/ground_truth_map");
  ASSERT_NE(map, nullptr) << "Failed to get gt occupancy grid";
  auto map_info = wombat_core::MapMetaDataAdapter(map->info);

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = 5.87773;
  start_pose.position.y = 1.33488;
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = 5.95814;
  end_pose.position.y = 1.3404;
  auto new_pose = kennel::apply_map_collisions(
    *map,
    start_pose,
    end_pose,
    1);
  auto maybe_new_pose_coord = wombat_core::world_pt_to_grid_coord(new_pose.position, map_info);
  EXPECT_NE(maybe_new_pose_coord, std::nullopt);
  EXPECT_EQ(maybe_new_pose_coord->x(), 295);
  EXPECT_EQ(maybe_new_pose_coord->y(), 67);
  auto maybe_new_pose_index = wombat_core::grid_coord_to_index(*maybe_new_pose_coord, map_info);
  EXPECT_NE(maybe_new_pose_index, std::nullopt);
  auto map_value = map->data[*maybe_new_pose_index];
  EXPECT_EQ(map_value, 0);
}

TEST_F(WallsWorldTest, WallCollision)
{
  auto map = robot->get_occupancy_grid(std::chrono::seconds(10), "/ground_truth_map");
  ASSERT_NE(map, nullptr) << "Failed to get gt occupancy grid";
  auto map_info = wombat_core::MapMetaDataAdapter(map->info);

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = 0.124311;
  start_pose.position.y = 1.0;
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = -0.0539415;
  end_pose.position.y = 1.0;
  auto new_pose = kennel::apply_map_collisions(
    *map,
    start_pose,
    end_pose,
    1);
  auto maybe_new_pose_coord = wombat_core::world_pt_to_grid_coord(new_pose.position, map_info);
  EXPECT_NE(maybe_new_pose_coord, std::nullopt);
  EXPECT_EQ(maybe_new_pose_coord->x(), 5);
  EXPECT_EQ(maybe_new_pose_coord->y(), 50);
  auto maybe_new_pose_index = wombat_core::grid_coord_to_index(*maybe_new_pose_coord, map_info);
  EXPECT_NE(maybe_new_pose_index, std::nullopt);
  auto map_value = map->data[*maybe_new_pose_index];
  EXPECT_EQ(map_value, 0);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
