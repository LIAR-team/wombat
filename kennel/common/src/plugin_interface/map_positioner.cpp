// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "kennel/common/plugin_interface/map_positioner.hpp"
#include "kennel/common/types.hpp"

namespace kennel
{

bool MapPositioner::initialize_positioner(
  const std::string & plugin_name,
  rclcpp::Node * parent_node,
  const std::string & params_prefix)
{
  bool base_success = this->initialize_plugin(
    plugin_name,
    parent_node,
    params_prefix);
  if (!base_success) {
    RCLCPP_WARN(this->get_logger(), "Failed to do basic setup");
    return false;
  }

  const bool params_success = this->declare_map_positioner_params(parent_node);
  if (!params_success) {
    RCLCPP_WARN(this->get_logger(), "Failed to setup map publisher parameters");
    return false;
  }

  m_rate_controller = std::make_unique<wombat_core::RateController>(
    std::chrono::milliseconds(get_parameter("pose_update_period_ms").get<int64_t>()),
    m_clock);

  auto map_topic_name = get_parameter("map_topic_name").get<std::string>();
  if (!map_topic_name.empty()) {
    m_map_pub = parent_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  }

  const bool post_init_success = this->post_init();
  if (!post_init_success) {
    RCLCPP_WARN(this->get_logger(), "Failed to run post-init routine");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Positioner initialized");
  return true;
}

std::optional<geometry_msgs::msg::TransformStamped> MapPositioner::positioner_update(
  const localization_data_t & gt_data)
{
  if (!m_rate_controller->is_ready()) {
    return std::nullopt;
  }

  auto positioner_data = compute_positioner_data(gt_data);
  if (m_map_pub && positioner_data.map) {
    m_map_pub->publish(*(positioner_data.map));
  }
  return positioner_data.robot_pose;
}

bool MapPositioner::declare_map_positioner_params(rclcpp::Node * parent_node)
{
  std::vector<default_parameter_info_t> params_info;
  default_parameter_info_t info;
  info.descriptor = rcl_interfaces::msg::ParameterDescriptor();

  info.name = "frame_id";
  info.value = rclcpp::ParameterValue(m_plugin_name);
  params_info.push_back(info);

  info.name = "map_topic_name";
  info.value = rclcpp::ParameterValue(m_plugin_name);
  params_info.push_back(info);

  info.name = "pose_update_period_ms";
  info.value = rclcpp::ParameterValue(20);
  params_info.push_back(info);

  const bool params_success = this->declare_parameters(
    params_info,
    parent_node);

  return params_success;
}

}  // namespace kennel
