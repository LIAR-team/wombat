// Copyright 2021-2022 Soragna Alberto.

#include <memory>

#include "wombat_hello_world/hello_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloPublisher>());
  rclcpp::shutdown();
  return 0;
}
