// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kennel/mobile_base/ground_truth_manager.hpp"
#include "kennel/mobile_base/mobile_base.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_core/math/transformations.hpp"

namespace kennel
{

MobileBase::MobileBase(rclcpp::Node * parent_node)
: m_parent_node(parent_node), m_logger(parent_node->get_logger())
{
  const bool gt_setup_success = this->setup_ground_truth();
  if (!gt_setup_success) {
    throw std::runtime_error("Failed to setup ground truth manager");
  }

  const bool slam_setup_success = this->setup_slam();
  if (!slam_setup_success) {
    throw std::runtime_error("Failed to setup SLAM manager");
  }

  const std::string control_topic_name = m_parent_node->declare_parameter(
    "mobile_base.control_topic_name",
    rclcpp::ParameterValue{std::string("cmd_vel")}).get<std::string>();

  m_last_cmd_vel.header.stamp = m_parent_node->now();
  m_cmd_vel_sub = m_parent_node->create_subscription<geometry_msgs::msg::Twist>(
    control_topic_name,
    rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      m_last_cmd_vel.header.stamp = m_parent_node->now();
      m_last_cmd_vel.twist = *msg;
    });

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_parent_node);

  const auto mobile_base_update_period = std::chrono::milliseconds(
    m_parent_node->declare_parameter(
      "mobile_base.update_period_ms",
      rclcpp::ParameterValue{10}).get<int>());

  // TODO: is wall time ok here? Should we use simulated time?
  m_update_timer = m_parent_node->create_wall_timer(
    mobile_base_update_period,
    [this]() {
      this->mobile_base_update();
    });

  RCLCPP_INFO(m_logger, "Mobile Base constructed");
}

localization_data_t MobileBase::get_ground_truth_data()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  localization_data_t data;
  data.robot_pose = m_gt_manager->get_pose();
  if (m_gt_map) {
    data.map = *m_gt_map;
  }
  return data;
}

void MobileBase::mobile_base_update()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_ground_truth_map_sub && !m_gt_map) {
    return;
  }
  auto gt_T_base = m_gt_manager->pose_update(m_last_cmd_vel, *m_gt_map);
  auto maybe_slam_data = m_slam_manager->slam_update(gt_T_base, *m_gt_map);

  std::vector<geometry_msgs::msg::TransformStamped> sorted_tfs;
  sorted_tfs.push_back(gt_T_base);

  // TODO: this is ugly, will need to rewrite.
  // We always need a full tree of transforms even if they are computed with different periods
  if (maybe_slam_data) {
    sorted_tfs.push_back(maybe_slam_data->robot_pose);
  } else {
    if (m_last_transforms.size() >= sorted_tfs.size() + 1) {
      sorted_tfs.push_back(m_last_transforms[1]);
    } else {
      return;
    }
  }

  auto maybe_tree_tfs = this->process_transforms(sorted_tfs);
  if (!maybe_tree_tfs) {
    throw std::runtime_error("Failed to process transforms!");
  }
  m_tf_broadcaster->sendTransform(*maybe_tree_tfs);
  m_last_transforms = *maybe_tree_tfs;

  if (m_slam_map_pub && maybe_slam_data) {
    m_slam_map_pub->publish(maybe_slam_data->map);
  }
}

std::optional<std::vector<geometry_msgs::msg::TransformStamped>>
MobileBase::process_transforms(
  const std::vector<geometry_msgs::msg::TransformStamped> & sorted_tfs)
{
  // Nothing to do if we don't have transforms
  if (sorted_tfs.empty()) {
    return std::nullopt;
  }

  // The transformations are assumed to be sorted and are required
  // to have the same child frame ID.
  // Given a_T_x (a transformation from frame "a" to frame "x") and b_T_x
  // (a transformation from frame "b" to frame "x"),
  // we want to compute a_T_b and b_T_x so that they can be concatenated in a tree structure.
  std::vector<geometry_msgs::msg::TransformStamped> tree_tfs;
  for (size_t i = 1; i < sorted_tfs.size(); i++) {
    auto maybe_composed_tf = wombat_core::compose_tfs(sorted_tfs[i - 1], sorted_tfs[i]);
    if (!maybe_composed_tf) {
      RCLCPP_ERROR(
        m_logger, "Failed to compose '%s -> %s' with '%s -> %s'",
        sorted_tfs[i - 1].header.frame_id.c_str(), sorted_tfs[i - 1].child_frame_id.c_str(),
        sorted_tfs[i].header.frame_id.c_str(), sorted_tfs[i].child_frame_id.c_str());
      return std::nullopt;
    }
    tree_tfs.push_back(*maybe_composed_tf);
  }
  // The last transformation can be used as-is
  tree_tfs.push_back(sorted_tfs.back());

  return tree_tfs;
}

bool MobileBase::setup_ground_truth()
{
  const std::string ground_truth_frame_id = m_parent_node->declare_parameter(
    "mobile_base.ground_truth_frame_id",
    rclcpp::ParameterValue{std::string("ground_truth")}).get<std::string>();
  const std::string robot_base_frame_id = m_parent_node->declare_parameter(
    "mobile_base.robot_base_frame_id",
    rclcpp::ParameterValue{std::string("base_link")}).get<std::string>();
  const std::string ground_truth_map_topic_name = m_parent_node->declare_parameter(
    "mobile_base.ground_truth_map_topic_name",
    rclcpp::ParameterValue{std::string("ground_truth_map")}).get<std::string>();
  const std::vector<double> start_pose_2d = m_parent_node->declare_parameter(
    "mobile_base.start_pose",
    rclcpp::ParameterValue{std::vector<double>({0.0, 0.0, 0.0})}).get<std::vector<double>>();
  const auto control_msg_lifespan = std::chrono::milliseconds(
    m_parent_node->declare_parameter(
      "mobile_base.control_msg_lifespan_ms",
      rclcpp::ParameterValue{100}).get<int>());

  if (start_pose_2d.size() != 3u) {
    RCLCPP_ERROR(
      m_logger,
      "Start pose has incorrect number of values: expected 3 got %d",
      static_cast<int>(start_pose_2d.size()));
    return false;
  }
  m_gt_manager = std::make_unique<GroundTruthManager>(
    m_parent_node,
    ground_truth_frame_id,
    control_msg_lifespan,
    robot_base_frame_id);

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = start_pose_2d[0];
  start_pose.position.y = start_pose_2d[1];
  start_pose.orientation = wombat_core::quaternion_from_rpy(0.0, 0.0, start_pose_2d[2]);
  m_gt_manager->reset_pose(start_pose);

  if (!ground_truth_map_topic_name.empty()) {
    RCLCPP_INFO(m_logger, "Creating ground truth map subscription: %s", ground_truth_map_topic_name.c_str());
    m_ground_truth_map_sub = m_parent_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      ground_truth_map_topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        RCLCPP_INFO(m_logger, "Received ground truth map");
        std::lock_guard<std::mutex> lock(m_mutex);
        m_gt_map = msg;
      });
  } else {
    m_gt_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }

  return true;
}

bool MobileBase::setup_slam()
{
  const std::string slam_frame_id = m_parent_node->declare_parameter(
    "mobile_base.slam_frame_id",
    rclcpp::ParameterValue{std::string("map")}).get<std::string>();
  const std::string slam_map_topic_name = m_parent_node->declare_parameter(
    "mobile_base.slam_map_topic_name",
    rclcpp::ParameterValue{std::string("map")}).get<std::string>();
  const auto slam_update_period = std::chrono::milliseconds(
    m_parent_node->declare_parameter(
      "mobile_base.slam_update_period_ms",
      rclcpp::ParameterValue{50}).get<int>());

  m_slam_manager = std::make_unique<SlamManager>(
    m_parent_node,
    rclcpp::Duration(slam_update_period),
    slam_frame_id);

  if (!slam_map_topic_name.empty()) {
    RCLCPP_INFO(m_logger, "Creating SLAM map publisher");
    m_slam_map_pub = m_parent_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      slam_map_topic_name,
      rclcpp::QoS(rclcpp::KeepLast(10)));
  }

  return true;
}

}  // namespace kennel
