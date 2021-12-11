// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include "wombat_hello_world/hello_publisher.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

HelloPublisher::HelloPublisher()
: Node("hello_publisher")
{
  m_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
  auto timer_callback =
    [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(this->m_count++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      this->m_publisher->publish(message);
    };
  m_timer = this->create_wall_timer(500ms, timer_callback);
}
