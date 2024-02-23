// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <memory>

#include "kennel/common/collisions.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/ros2/parameters.hpp"

#include "kennel/kennel_gtest/single_robot_fixture.hpp"
#include "kennel/kennel_gtest/utils.hpp"

class WallsWorldTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();
    rclcpp::ParameterMap parameter_map;
    ASSERT_NO_THROW(parameter_map = rclcpp::parameter_map_from_yaml_file(get_data_path("single_robot.yaml")));

    bool success = wombat_core::write_parameter_map(
      parameter_map,
      "/kennel",
      "map_yaml_filename",
      rclcpp::ParameterValue(get_data_path("walls_map.yaml")));
    ASSERT_TRUE(success);

    setup_kennel(parameter_map);
  }
};

TEST_F(WallsWorldTest, WallCollision)
{
  auto map = this->get_occupancy_grid();
  auto map_info = wombat_core::MapMetaDataAdapter(map->info);
  ASSERT_NE(map, nullptr);

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = 0.124311;
  start_pose.position.y = 1.0;
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = -0.0539415;
  end_pose.position.y = 1.0;
  auto new_pose = kennel::apply_map_collisions(
    *map,
    start_pose,
    end_pose);
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
