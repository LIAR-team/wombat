// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel_gtest/kennel_config.hpp"
#include "kennel/kennel_gtest/single_robot_fixture.hpp"
#include "kennel/kennel_gtest/utils.hpp"

class MultiPositionerTestNoMap : public TestKennelSingleRobot
{
public:
  void SetUp() override
  {
    TestKennelSingleRobot::SetUp();

    auto kennel_params = kennel::KennelParamsConfig()
      .add_robot()
      .set_robot_pose({3.0, -1.0, 0.0})
      .add_lidar_slam_positioner_to_robot(
      kennel::DEFAULT_ROBOT_NODE,
      kennel::NamedParams({}),
      "map")
      .add_stub_positioner_to_robot(
      kennel::DEFAULT_ROBOT_NODE,
      kennel::NamedParams({}),
      "odom")
      .set_map_yaml_filename("")
      .get();
    setup_kennel(kennel_params);
  }
};

TEST_F(MultiPositionerTestNoMap, CheckPose)
{
  auto maybe_tf = robot->get_latest_base_tf(std::chrono::seconds(10));
  EXPECT_NE(maybe_tf, std::nullopt);

  EXPECT_NEAR(maybe_tf->transform.translation.x, 3.0, 1e-5);
  EXPECT_NEAR(maybe_tf->transform.translation.y, -1.0, 1e-5);
  EXPECT_NEAR(maybe_tf->transform.translation.z, 0.0, 1e-5);
  EXPECT_NEAR(maybe_tf->transform.rotation.x, 0.0, 1e-5);
  EXPECT_NEAR(maybe_tf->transform.rotation.y, 0.0, 1e-5);
  EXPECT_NEAR(maybe_tf->transform.rotation.z, 0.0, 1e-5);
  EXPECT_NEAR(maybe_tf->transform.rotation.w, 1.0, 1e-5);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
