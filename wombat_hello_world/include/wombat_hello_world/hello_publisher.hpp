// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#ifndef WOMBAT_HELLO_WORLD__HELLO_PUBLISHER_HPP_
#define WOMBAT_HELLO_WORLD__HELLO_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace wombat_hello_world
{

class HelloPublisher : public rclcpp::Node
{
public:
  explicit HelloPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
  size_t m_count {0};
};

}  // namespace wombat_hello_world

#endif  // WOMBAT_HELLO_WORLD__HELLO_PUBLISHER_HPP_
