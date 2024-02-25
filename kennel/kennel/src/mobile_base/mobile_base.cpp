// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kennel/mobile_base/ground_truth_manager.hpp"
#include "kennel/mobile_base/mobile_base.hpp"
#include "wombat_core/costmap/costmap_conversions.hpp"
#include "wombat_core/math/angles.hpp"
#include "wombat_core/math/transformations.hpp"
#include "wombat_core/ros2/parameters.hpp"

// Macros to generate nested parameter names
#include "wombat_core/cpp/macros/concat.hpp"
#define BASE_NAME "mobile_base"
#define BASE_PREFIX  BASE_NAME "."
#define BASE_PARAM(PARAM) WOMBAT_CORE_CONCAT_STR_LITERALS(BASE_PREFIX, PARAM)
#define BASE_GT_PARAM(PARAM) BASE_PARAM(WOMBAT_CORE_CONCAT_STR_LITERALS("ground_truth.", PARAM))

namespace kennel
{

MobileBase::MobileBase(rclcpp::Node * parent_node)
: m_parent_node(parent_node), m_logger(parent_node->get_logger()), m_clock(parent_node->get_clock())
{
  const bool gt_setup_success = this->setup_ground_truth();
  if (!gt_setup_success) {
    throw std::runtime_error("Failed to setup ground truth manager");
  }

  const bool plugins_success = this->load_positioner_plugins(parent_node);
  if (!plugins_success) {
    throw std::runtime_error("Failed to register positioner plugins");
  }

  const std::string control_topic_name = wombat_core::declare_parameter_if_not_declared(
    m_parent_node->get_node_parameters_interface(),
    BASE_PARAM("control_topic_name"),
    rclcpp::ParameterValue{std::string("/cmd_vel")}).get<std::string>();

  m_last_cmd_vel.header.stamp = m_parent_node->now();
  m_cmd_vel_sub = m_parent_node->create_subscription<geometry_msgs::msg::Twist>(
    control_topic_name,
    rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      m_last_cmd_vel.header.stamp = m_parent_node->now();
      m_last_cmd_vel.twist = *msg;
    });

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_parent_node);

  m_update_period_ms = std::chrono::milliseconds(
    wombat_core::declare_parameter_if_not_declared(
      m_parent_node->get_node_parameters_interface(),
      BASE_PARAM("update_period_ms"),
      rclcpp::ParameterValue{10}).get<int>());

  RCLCPP_INFO(m_logger, "Mobile Base constructed");
}

std::chrono::milliseconds MobileBase::get_update_period() const
{
  return m_update_period_ms;
}

localization_data_t MobileBase::get_ground_truth_data()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  localization_data_t data;
  data.robot_pose = m_gt_manager->get_pose();
  data.map = m_gt_map;
  return data;
}

void MobileBase::mobile_base_update()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_ground_truth_map_sub && !m_gt_map) {
    RCLCPP_WARN_THROTTLE(
      m_logger, *m_clock, 1000,
      "Waiting for gt map %s",
      m_ground_truth_map_sub->get_topic_name());
    return;
  }
  kennel::localization_data_t gt_data;
  gt_data.robot_pose = m_gt_manager->pose_update(m_last_cmd_vel);
  gt_data.map = m_gt_map;

  std::vector<geometry_msgs::msg::TransformStamped> sorted_tfs;
  sorted_tfs.push_back(gt_data.robot_pose);

  for (size_t i = 0; i < m_positioners.size(); i++) {
    auto maybe_tf = m_positioners[i]->positioner_update(gt_data);
    // TODO: this is ugly, will need to rewrite.
    // We always need a full tree of transforms even if they are computed with different periods
    if (maybe_tf) {
      sorted_tfs.push_back(*maybe_tf);
    } else {
      size_t sorted_positioner_id = i + 1;
      if (m_last_transforms.size() >= sorted_positioner_id) {
        sorted_tfs.push_back(m_last_transforms[sorted_positioner_id]);
      } else {
        RCLCPP_WARN_THROTTLE(
          m_logger, *m_clock, 1000,
          "Failed to sort tf %d",
          static_cast<int>(i));
        return;
      }
    }
  }

  auto maybe_tree_tfs = this->process_transforms(sorted_tfs);
  if (!maybe_tree_tfs) {
    throw std::runtime_error("Failed to process transforms!");
  }
  m_tf_broadcaster->sendTransform(*maybe_tree_tfs);
  m_last_transforms = *maybe_tree_tfs;
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
  const auto ground_truth_frame_id = wombat_core::declare_parameter_if_not_declared(
    m_parent_node->get_node_parameters_interface(),
    BASE_GT_PARAM("frame_id"),
    rclcpp::ParameterValue{std::string("ground_truth")}).get<std::string>();

  const auto ground_truth_costmap_topic_name = wombat_core::declare_parameter_if_not_declared(
    m_parent_node->get_node_parameters_interface(),
    BASE_GT_PARAM("costmap_topic_name"),
    rclcpp::ParameterValue{std::string("ground_truth_costmap")}).get<std::string>();

  const auto input_ground_truth_map_topic_name = wombat_core::declare_parameter_if_not_declared(
    m_parent_node->get_node_parameters_interface(),
    BASE_GT_PARAM("map_topic_name"),
    rclcpp::ParameterValue{std::string("/ground_truth_map")}).get<std::string>();

  const auto robot_base_frame_id = wombat_core::declare_parameter_if_not_declared(
    m_parent_node->get_node_parameters_interface(),
    BASE_PARAM("frame_id"),
    rclcpp::ParameterValue{std::string("base_link")}).get<std::string>();

  const auto start_pose_2d = wombat_core::declare_parameter_if_not_declared(
    m_parent_node->get_node_parameters_interface(),
    BASE_PARAM("start_pose"),
    rclcpp::ParameterValue{std::vector<double>({0.0, 0.0, 0.0})}).get<std::vector<double>>();

  const auto control_msg_lifespan = std::chrono::milliseconds(
    wombat_core::declare_parameter_if_not_declared(
      m_parent_node->get_node_parameters_interface(),
      BASE_PARAM("control_msg_lifespan_ms"),
      rclcpp::ParameterValue{100}).get<int>());

  const auto robot_radius = wombat_core::declare_parameter_if_not_declared(
    m_parent_node->get_node_parameters_interface(),
    BASE_PARAM("robot_radius"),
    rclcpp::ParameterValue{0.0}).get<double>();

  if (start_pose_2d.size() != 3u) {
    RCLCPP_ERROR(
      m_logger,
      "Start pose has incorrect number of values: expected 3 got %d",
      static_cast<int>(start_pose_2d.size()));
    return false;
  }
  RCLCPP_INFO(m_logger, "Start pose: %f %f %f", start_pose_2d[0], start_pose_2d[1], start_pose_2d[2]);
  m_gt_manager = std::make_unique<GroundTruthManager>(
    m_parent_node,
    ground_truth_frame_id,
    control_msg_lifespan,
    robot_radius,
    robot_base_frame_id);
  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = start_pose_2d[0];
  start_pose.position.y = start_pose_2d[1];
  start_pose.orientation = wombat_core::quaternion_from_rpy(0.0, 0.0, start_pose_2d[2]);
  m_gt_manager->reset_pose(start_pose);

  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  if (!input_ground_truth_map_topic_name.empty()) {
    RCLCPP_INFO(m_logger, "Creating ground truth map subscription: %s", input_ground_truth_map_topic_name.c_str());
    rclcpp::SubscriptionOptions sub_options;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    m_ground_truth_map_sub = m_parent_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_ground_truth_map_topic_name,
      map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        this->on_gt_map_received(std::move(msg));
      },
      sub_options);
  }

  if (!ground_truth_costmap_topic_name.empty()) {
    if (input_ground_truth_map_topic_name.empty()) {
      RCLCPP_WARN(
        m_logger,
        "Can't construct costmap publisher for %s without a ground_truth map subscrition",
        ground_truth_costmap_topic_name.c_str());
    } else {
      rclcpp::PublisherOptions pub_options;
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
      m_gt_costmap_pub = m_parent_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        ground_truth_costmap_topic_name,
        map_qos,
        pub_options);
      RCLCPP_INFO(m_logger, "Created ground truth costmap publisher %s", m_gt_costmap_pub->get_topic_name());
    }
  }

  return true;
}

void MobileBase::on_gt_map_received(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  RCLCPP_INFO(m_logger, "Received ground truth map");
  m_gt_manager->map_update(msg);
  if (m_gt_costmap_pub) {
    auto costmap = m_gt_manager->get_costmap();
    auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    grid->header.frame_id = m_gt_manager->get_ground_truth_frame_id();
    grid->header.stamp = m_clock->now();
    wombat_core::costmap_to_occupancy_grid_values(*costmap, *grid);
    RCLCPP_INFO(m_logger, "Published costmap");
    m_gt_costmap_pub->publish(std::move(grid));
  }
  std::lock_guard<std::mutex> lock(m_mutex);
  m_gt_map = msg;
}

bool MobileBase::load_positioner_plugins(rclcpp::Node * parent_node)
{
  // Get name of plugins to load
  auto positioner_plugins = wombat_core::declare_parameter_if_not_declared(
    parent_node->get_node_parameters_interface(),
    BASE_PARAM("positioners"),
    rclcpp::ParameterValue(std::vector<std::string>())).get<std::vector<std::string>>();

  std::vector<std::shared_ptr<PositionerInterface>> new_positioners;
  for (const auto & plugin_name : positioner_plugins) {
    // Get type of this plugin
    auto plugin_type = wombat_core::declare_parameter_if_not_declared(
      parent_node->get_node_parameters_interface(),
      BASE_PREFIX + plugin_name + ".plugin_type",
      rclcpp::ParameterValue("")).get<std::string>();
    if (plugin_type.empty()) {
      RCLCPP_ERROR(m_logger, "Plugin %s has no plugin type defined", plugin_name.c_str());
      return false;
    }

    // Load and initialize the plugin
    auto loaded_plugin = m_plugin_loader.createSharedInstance(plugin_type);
    const bool plugin_init_success = loaded_plugin->initialize_positioner(
      plugin_name,
      parent_node,
      BASE_NAME);
    if (!plugin_init_success) {
      RCLCPP_WARN(m_logger, "Failed to initialize plugin %s", plugin_name.c_str());
      return false;
    }

    // Store the initialized plugin
    new_positioners.push_back(std::move(loaded_plugin));
  }

  m_positioners = new_positioners;
  return true;
}

}  // namespace kennel
