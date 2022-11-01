// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>
#include <iostream>
#include <memory>

#include "wombat_hello_world/hello_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(test_hello, hello_publisher)
{
  rclcpp::init(0, nullptr);
  auto publisher = std::make_shared<HelloPublisher>();
  std::cout << "Hello publisher unit test!" << std::endl;
  rclcpp::shutdown();

  EXPECT_EQ(0, 0);
}
