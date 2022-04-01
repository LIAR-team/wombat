// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include "rclcpp/rclcpp.hpp"
#include "wombat_strategy/exploration/frontier_exploration_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<wombat_strategy::FrontierExplorationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
