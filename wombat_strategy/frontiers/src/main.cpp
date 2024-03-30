// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "rclcpp/rclcpp.hpp"
#include "wombat_strategy/frontiers/frontier_navigation_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<wombat_strategy::FrontiersNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
