// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <memory>

#include "kennel/common/sensors/range_projection.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_core/ros2/parameters.hpp"

#include "single_robot_fixture.hpp"
#include "utils.hpp"

class WallsWorldTest : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();
    rclcpp::ParameterMap parameter_map;
    ASSERT_NO_THROW(parameter_map = rclcpp::parameter_map_from_yaml_file(get_data_path("single_robot.yaml")));

    bool success = wombat_core::update_parameter_map(
      parameter_map,
      "/kennel",
      "map_yaml_filename",
      rclcpp::ParameterValue(get_data_path("walls_map.yaml")));
    ASSERT_TRUE(success);

    setup_kennel(parameter_map);
  }
};

TEST_F(WallsWorldTest, ShortRangeProjection)
{
  auto map = this->get_occupancy_grid();
  ASSERT_NE(map, nullptr);

  geometry_msgs::msg::Pose laser_pose;
  laser_pose.position.x = 2.0;
  laser_pose.position.y = 2.0;

  size_t num_bins = 10;
  auto ranges = kennel::compute_laser_ranges(
    *map,
    laser_pose,
    num_bins,
    std::make_pair(wombat_core::rad_to_deg(-20.0), wombat_core::rad_to_deg(20.0)),
    std::make_pair(0.0, 0.2));

  ASSERT_EQ(ranges.size(), num_bins);
  for (const auto & r : ranges) {
    EXPECT_EQ(r, 0.2);
  }
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
