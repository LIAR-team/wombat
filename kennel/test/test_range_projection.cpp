// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <memory>

#include "kennel/common/range_projection.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/math/angles.hpp"
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

    setup_kennel(kennel_params);
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
    std::make_pair(wombat_core::deg_to_rad(-20.0), wombat_core::deg_to_rad(20.0)),
    std::make_pair(0.0, 0.2));

  ASSERT_EQ(ranges.size(), num_bins);
  for (const auto & r : ranges) {
    EXPECT_LE(std::abs(r - 0.2), map->info.resolution * 2);
  }
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
