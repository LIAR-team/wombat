// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include "wombat_core/ros2/parameters.hpp"

TEST(TestRos2Parameters, ParametersDeclaration)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("my_node");

  const std::string param_name = "test_parameter";
  // Ensure that the parameter doesn't exist already
  EXPECT_FALSE(node->has_parameter(param_name));

  const std::string default_value = "default_value";
  rclcpp::ParameterValue value = wombat_core::declare_parameter_if_not_declared(
    node->get_node_parameters_interface(),
    param_name,
    rclcpp::ParameterValue(default_value));

  // The parameter now exists and has the default value
  EXPECT_TRUE(node->has_parameter(param_name));
  EXPECT_EQ(value.get_type(), rclcpp::PARAMETER_STRING);
  EXPECT_EQ(value.get<std::string>(), default_value);

  // Change the value of the parameter
  const std::string set_value = "set_value";
  node->set_parameter(
    rclcpp::Parameter(
      param_name,
      rclcpp::ParameterValue(set_value)));

  // Try to re-declare the parameter with the default value
  rclcpp::ParameterValue new_value = wombat_core::declare_parameter_if_not_declared(
    node->get_node_parameters_interface(),
    param_name,
    rclcpp::ParameterValue(default_value));

  // The parameter still exists and has the set value
  EXPECT_TRUE(node->has_parameter(param_name));
  EXPECT_EQ(new_value.get_type(), rclcpp::PARAMETER_STRING);
  EXPECT_EQ(new_value.get<std::string>(), set_value);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
