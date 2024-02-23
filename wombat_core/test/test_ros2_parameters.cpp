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

TEST(TestRos2Parameters, UpdateParameterMap)
{
  static constexpr auto node1_fqn = "/my_node";
  static constexpr auto node2_fqn = "/my_space/my_node";

  static constexpr auto param1_name = "my_param1";
  static constexpr auto param2_name = "my_param2";

  rclcpp::ParameterMap parameter_map;
  bool result = false;
  std::optional<rclcpp::Parameter> param;

  // Check that parameters are not set
  param = wombat_core::get_parameter_for_node(param1_name, parameter_map, node1_fqn);
  ASSERT_EQ(param, std::nullopt);
  param = wombat_core::get_parameter_for_node(param2_name, parameter_map, node1_fqn);
  ASSERT_EQ(param, std::nullopt);
  param = wombat_core::get_parameter_for_node(param1_name, parameter_map, node2_fqn);
  ASSERT_EQ(param, std::nullopt);
  param = wombat_core::get_parameter_for_node(param2_name, parameter_map, node2_fqn);
  ASSERT_EQ(param, std::nullopt);

  // Set param1 for node 1
  result = wombat_core::write_parameter_map(
    parameter_map,
    node1_fqn,
    param1_name,
    rclcpp::ParameterValue(1),
    false);
  EXPECT_TRUE(result);
  param = wombat_core::get_parameter_for_node(param1_name, parameter_map, node1_fqn);
  EXPECT_NE(param, std::nullopt);
  EXPECT_EQ(param->as_int(), 1);

  // Override param1 for node 1
  result = wombat_core::write_parameter_map(
    parameter_map,
    node1_fqn,
    param1_name,
    rclcpp::ParameterValue(2),
    true);
  EXPECT_TRUE(result);
  param = wombat_core::get_parameter_for_node(param1_name, parameter_map, node1_fqn);
  EXPECT_NE(param, std::nullopt);
  EXPECT_EQ(param->as_int(), 2);

  // Try to set again param1 for node 1 but fail without override
  result = wombat_core::write_parameter_map(
    parameter_map,
    node1_fqn,
    param1_name,
    rclcpp::ParameterValue(9999),
    false);
  EXPECT_FALSE(result);
  param = wombat_core::get_parameter_for_node(param1_name, parameter_map, node1_fqn);
  EXPECT_NE(param, std::nullopt);
  EXPECT_EQ(param->as_int(), 2);

  // Set param2 for node2
  result = wombat_core::write_parameter_map(
    parameter_map,
    node2_fqn,
    param2_name,
    rclcpp::ParameterValue(1),
    false);
  EXPECT_TRUE(result);
  param = wombat_core::get_parameter_for_node(param2_name, parameter_map, node2_fqn);
  EXPECT_NE(param, std::nullopt);
  EXPECT_EQ(param->as_int(), 1);

  // Override param2 for
  result = wombat_core::write_parameter_map(
    parameter_map,
    node2_fqn,
    param2_name,
    rclcpp::ParameterValue(2),
    true);
  EXPECT_TRUE(result);
  param = wombat_core::get_parameter_for_node(param2_name, parameter_map, node2_fqn);
  EXPECT_NE(param, std::nullopt);
  EXPECT_EQ(param->as_int(), 2);

  // Set param2 for node1
  result = wombat_core::write_parameter_map(
    parameter_map,
    node1_fqn,
    param2_name,
    rclcpp::ParameterValue(42),
    false);
  EXPECT_TRUE(result);
  param = wombat_core::get_parameter_for_node(param2_name, parameter_map, node1_fqn);
  EXPECT_NE(param, std::nullopt);
  EXPECT_EQ(param->as_int(), 42);

  param = wombat_core::get_parameter_for_node(param2_name, parameter_map, node2_fqn);
  EXPECT_NE(param, std::nullopt);
  EXPECT_EQ(param->as_int(), 2);
}

TEST(TestRos2Parameters, SetParameterMap)
{
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  auto node = std::make_shared<rclcpp::Node>("my_node", node_options);
  static constexpr auto param1_name = "my_param1";

  rclcpp::ParameterMap parameter_map;
  bool result = false;
  result = wombat_core::write_parameter_map(
    parameter_map,
    node->get_fully_qualified_name(),
    param1_name,
    rclcpp::ParameterValue(1),
    true);
  EXPECT_TRUE(result);
  result = wombat_core::write_parameter_map(
    parameter_map,
    node->get_fully_qualified_name(),
    param1_name,
    rclcpp::ParameterValue(2),
    true);
  EXPECT_TRUE(result);

  result = wombat_core::set_parameters_from_map(
    parameter_map,
    node->get_node_base_interface(),
    node->get_node_parameters_interface());
  EXPECT_TRUE(result);

  auto param = node->get_parameter(param1_name);
  EXPECT_EQ(param.as_int(), 2);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
