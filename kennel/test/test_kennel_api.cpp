// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel.hpp"
#include "kennel/kennel_gtest/utils.hpp"

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
  bool start_success = kennel->start();
  ASSERT_TRUE(start_success);
  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);
}

TEST_F(TestKennelRos, stop_without_start)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);
}

TEST_F(TestKennelRos, start_multiple_times)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  bool start_success = kennel->start();
  ASSERT_TRUE(start_success);
  start_success = kennel->start();
  ASSERT_TRUE(start_success);
  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);
}

TEST_F(TestKennelRos, restarting_kennel)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  bool start_success = kennel->start();
  ASSERT_TRUE(start_success);
  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);
  start_success = kennel->start();
  ASSERT_TRUE(start_success);
  stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);
}

TEST_F(TestKennelRos, shutdown_before_stop)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  bool start_success = kennel->start();
  ASSERT_TRUE(start_success);
  rclcpp::shutdown();
  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);
}

TEST_F(TestKennelRos, configure)
{
  auto kennel = std::make_unique<kennel::Kennel>();
  bool load_success = kennel->configure();
  ASSERT_TRUE(load_success);
  bool start_success = kennel->start();
  ASSERT_TRUE(start_success);
  bool stop_success = kennel->stop();
  ASSERT_TRUE(stop_success);
}

int main(int argc, char ** argv)
{
  setup_data_dir_path();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
