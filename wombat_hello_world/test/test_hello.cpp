// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include <gtest/gtest.h>
#include <iostream>
#include <memory>

#include "wombat_hello_world/hello_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(test_hello, hello_publisher)
{
  rclcpp::init(0, nullptr);
  auto publisher = std::make_shared<wombat_hello_world::HelloPublisher>();
  std::cout << "Hello publisher unit test!" << std::endl;
  rclcpp::shutdown();

  EXPECT_EQ(0, 0);
}
