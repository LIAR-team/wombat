// Copyright 2021-2022 Soragna Alberto.

#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloPublisher : public rclcpp::Node
{
public:
  explicit HelloPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
  size_t m_count {0};
};
