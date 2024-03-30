// Copyright 2024 Soragna Alberto.

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
    rclcpp::PublisherOptions pub_options;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    m_map_pub = parent_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      pub_options);
    RCLCPP_INFO(this->get_logger(), "Publishing occupancy grid msg on %s", m_map_pub->get_topic_name());
  }

  auto odometry_topic_name = get_parameter("odometry_topic_name").get<std::string>();
  if (!odometry_topic_name.empty()) {
    m_odom_pub = parent_node->create_publisher<nav_msgs::msg::Odometry>(
      odometry_topic_name,
      rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Publishing odometry msg on %s", m_odom_pub->get_topic_name());
  }

  const bool post_init_success = this->post_init();
  if (!post_init_success) {
    RCLCPP_WARN(this->get_logger(), "Failed to run post-init routine");
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "Positioner %s initialized: map %s frame %s",
    plugin_name.c_str(),
    (m_map_pub ? m_map_pub->get_topic_name() : "[N/A]"),
    this->get_parameter("frame_id").get<std::string>().c_str());
  return true;
}

std::optional<geometry_msgs::msg::TransformStamped> MapPositioner::positioner_update(
  const localization_data_t & gt_data,
  const geometry_msgs::msg::TwistStamped & cmd_vel)
{
  if (!m_rate_controller->is_ready()) {
    return std::nullopt;
  }

  auto positioner_data = compute_positioner_data(gt_data);
  if (m_map_pub && positioner_data.map) {
    m_map_pub->publish(*(positioner_data.map));
  }

  const auto & robot_tf = positioner_data.robot_pose;
  if (m_odom_pub) {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header = robot_tf.header;
    odom_msg->child_frame_id = robot_tf.child_frame_id;
    odom_msg->pose.pose.position.x = robot_tf.transform.translation.x;
    odom_msg->pose.pose.position.y = robot_tf.transform.translation.y;
    odom_msg->pose.pose.position.z = robot_tf.transform.translation.z;
    odom_msg->pose.pose.orientation = robot_tf.transform.rotation;
    odom_msg->twist.twist = cmd_vel.twist;

    m_odom_pub->publish(std::move(odom_msg));
  }

  return robot_tf;
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

  info.name = "odometry_topic_name";
  info.value = rclcpp::ParameterValue("");
  params_info.push_back(info);

  const bool params_success = this->declare_parameters(
    params_info,
    parent_node);

  return params_success;
}

}  // namespace kennel
