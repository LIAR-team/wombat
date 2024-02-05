// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel.hpp"
#include "utils.hpp"

class TestKennelRos : public testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestKennelRos, default_start_stop)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  kennel->start();
  kennel->stop();
}

TEST_F(TestKennelRos, stop_without_start)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  kennel->stop();
}

TEST_F(TestKennelRos, start_multiple_times)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  kennel->start();
  kennel->start();
  kennel->stop();
}

TEST_F(TestKennelRos, restarting_kennel)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  kennel->start();
  kennel->stop();
  kennel->start();
  kennel->stop();
}

TEST_F(TestKennelRos, shutdown_before_stop)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  kennel->start();
  rclcpp::shutdown();
  kennel->stop();
}

TEST_F(TestKennelRos, configure_yaml)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  bool load_success = kennel->configure(get_data_path("empty_world.yaml"));
  ASSERT_TRUE(load_success);
  kennel->start();
  kennel->stop();
}

TEST_F(TestKennelRos, non_existant_parameters)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  bool load_success = kennel->configure(get_data_path("this_does_not_exist"));
  ASSERT_FALSE(load_success);
  kennel->start();
  kennel->stop();
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
