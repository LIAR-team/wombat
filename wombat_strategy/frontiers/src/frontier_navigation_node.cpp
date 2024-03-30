// Copyright 2021-2024 Soragna Alberto.

#include "wombat_strategy/frontiers/frontier_navigation_node.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "wombat_core/math/angles.hpp"
#include "wombat_core/math/geometry_point.hpp"
#include "wombat_core/ros2/parameters.hpp"

namespace wombat_strategy
{

FrontiersNavigationNode::FrontiersNavigationNode(const rclcpp::NodeOptions & options)
: Node("frontiers_navigation", options)
{
  // Node parameters
  m_global_frame = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "global_frame",
    rclcpp::ParameterValue("map")).get<std::string>();
  m_robot_frame = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "robot_frame",
    rclcpp::ParameterValue("base_link")).get<std::string>();
  auto map_topic = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "map_topic",
    rclcpp::ParameterValue("map")).get<std::string>();
  m_exploration_rate = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "exploration_rate",
    rclcpp::ParameterValue(5.0)).get<double>();
  m_min_attempted_goals_distance = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "min_attempted_goals_distance",
    rclcpp::ParameterValue(0.2)).get<double>();
  auto search_only_free_space = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "search_only_free_space",
    rclcpp::ParameterValue(true)).get<bool>();
  auto min_frontier_size = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "min_frontier_size",
    rclcpp::ParameterValue(10)).get<int>();
  auto frontier_size_scaling_factor = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "frontier_size_scaling_factor",
    rclcpp::ParameterValue(0.0)).get<double>();
  auto no_progress_seconds = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "no_progress_seconds",
    rclcpp::ParameterValue(10)).get<int>();
  bool publish_frontiers = wombat_core::declare_parameter_if_not_declared(
    this->get_node_parameters_interface(),
    "publish_frontiers",
    rclcpp::ParameterValue(true)).get<bool>();

  m_map_subscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic,
    10,
    std::bind(&FrontiersNavigationNode::map_callback, this, std::placeholders::_1));

  if (publish_frontiers) {
    m_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 10);
  }

  // Server implemented by this node
  m_explore_server = std::make_shared<FrontierActionServer>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "explore",
    std::bind(&FrontiersNavigationNode::explore, this));
  m_explore_server->activate();

  // Nav2 Navigation action client
  m_navigation_client = std::make_unique<NavigationClient>(
    std::make_shared<wombat_core::NodeInterfaces>(this),
    rclcpp::Duration(no_progress_seconds, 0));

  // TF buffer and Listener
  m_tf = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf->setUsingDedicatedThread(true);
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf, this, false);

  FrontierDetector::params_t params;
  params.search_only_free_space = search_only_free_space;
  params.min_frontier_size = min_frontier_size;
  params.frontier_size_scaling_factor = frontier_size_scaling_factor;
  m_detector = FrontierDetector(params);

  RCLCPP_INFO(this->get_logger(), "Node created");
}

FrontiersNavigationNode::~FrontiersNavigationNode()
{
  auto result = std::make_shared<typename FrontierAction::Result>();
  m_explore_server->terminate_all(result);
}

bool FrontiersNavigationNode::handle_cancellation(const rclcpp::Time & start_time)
{
  if (!m_explore_server->is_cancel_requested()) {
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Canceling frontiers navigation");
  auto result = std::make_shared<typename FrontierAction::Result>();
  result->total_elapsed_time = this->now() - start_time;
  m_explore_server->terminate_all(result);
  m_navigation_client->cancel_navigate_to_pose(rclcpp::Duration::from_seconds(1.0));

  return true;
}

void FrontiersNavigationNode::explore()
{
  auto goal = m_explore_server->get_current_goal();
  RCLCPP_INFO(this->get_logger(), "Starting navigation for %s frontiers!", (goal->explore_all ? "all" : "1"));

  auto start_time = this->now();
  m_strategy_state = StrategyState::START_NAVIGATION;

  rclcpp::Rate loop_rate(m_exploration_rate);
  while (rclcpp::ok()) {
    bool is_strategy_done = handle_cancellation(start_time);
    if (is_strategy_done) {
      break;
    }

    auto next_state = StrategyState::NONE;
    switch(m_strategy_state) {
    case StrategyState::DONE:
    [[fallthrough]];
    case StrategyState::NONE:
    {
      throw std::runtime_error("Bad strategy state");
    }
    case StrategyState::START_NAVIGATION:
    {
      bool has_next_goal = start_navigation();
      next_state = has_next_goal ? StrategyState::NAVIGATE_TO_FRONTIER : StrategyState::DONE;
      break;
    }
    case StrategyState::NAVIGATE_TO_FRONTIER:
    {
      bool is_navigation_done = handle_navigation();
      if (!is_navigation_done) {
        next_state = StrategyState::NAVIGATE_TO_FRONTIER;
        break;
      }
      // Navigation completed, should restart or terminate?
      next_state = goal->explore_all ? StrategyState::START_NAVIGATION : StrategyState::DONE;
      break;
    }
    }

    if (next_state == StrategyState::DONE) {
      auto result = std::make_shared<typename FrontierAction::Result>();
      result->total_elapsed_time = this->now() - start_time;
      m_explore_server->succeeded_current(result);
      RCLCPP_INFO(this->get_logger(), "Frontiers navigation completed with success!");
      break;
    }

    m_strategy_state = next_state;
    loop_rate.sleep();
  }
}

bool FrontiersNavigationNode::start_navigation()
{
  // Get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(current_pose, *m_tf, m_global_frame, m_robot_frame);

  // Search new frontiers
  const auto begin_search_frontier_wall_time = std::chrono::high_resolution_clock::now();
  auto frontiers = m_detector.search_frontiers(m_occupancy_grid, current_pose.pose.position);
  const auto end_search_frontier_wall_time = std::chrono::high_resolution_clock::now();

  const auto search_frontier_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_search_frontier_wall_time - begin_search_frontier_wall_time);
  RCLCPP_INFO(
    this->get_logger(), "Found %lu frontiers in %ld ms",
    frontiers.size(),
    search_frontier_duration_ms.count());

  // Select exploration goal
  auto maybe_navigation_goal = select_navigation_goal(frontiers);
  if (!maybe_navigation_goal) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Failed to select goal");
    return false;
  }
  m_curr_navigation_goal = *maybe_navigation_goal;
  RCLCPP_INFO_STREAM(this->get_logger(), "New goal: " << m_curr_navigation_goal.position.x << " " << m_curr_navigation_goal.position.y);
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose = m_curr_navigation_goal;
  goal_pose.header.frame_id = m_global_frame;
  m_navigation_client->start_navigate_to_pose(goal_pose);

  // Debugging visualization tools
  if (m_markers_pub) {
    visualize_frontiers(m_curr_navigation_goal, frontiers);
  }

  return true;
}

bool FrontiersNavigationNode::handle_navigation()
{
  bool valid = m_detector.validate_frontier(
    m_target_frontier,
    m_occupancy_grid,
    wombat_core::MapMetaDataAdapter(m_occupancy_grid->info));

  if (!valid) {
    RCLCPP_INFO(this->get_logger(), "Current frontier is not valid anymore, cancelling navigation");
    m_navigation_client->cancel_navigate_to_pose(rclcpp::Duration::from_seconds(1.0));
    return true;
  }

  // Get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  nav2_util::getCurrentPose(current_pose, *m_tf, m_global_frame, m_robot_frame);

  // Check the navigation status
  auto nav_status = m_navigation_client->handle_navigate_to_pose(current_pose);
  switch (nav_status.status) {
  case NavigationClient::Status::IDLE:
    throw std::runtime_error("received bad navigation status");
  case NavigationClient::Status::RUNNING:
    return false;
  case NavigationClient::Status::CANCELED:
    RCLCPP_INFO(this->get_logger(), "Navigate to pose action canceled");
    break;
  case NavigationClient::Status::DONE:
    RCLCPP_INFO(this->get_logger(), "Navigate to pose action completed");
    break;
  }

  m_attempted_goals.push_back(m_curr_navigation_goal);
  return true;
}

std::optional<geometry_msgs::msg::Pose>
FrontiersNavigationNode::select_navigation_goal(
  const std::vector<frontier_t> & frontiers)
{
  // Frontiers are already sorted according to score.
  // We compute a goal location from each sorted frontiers and we select the first valid one.
  const double min_dist_squared = m_min_attempted_goals_distance * m_min_attempted_goals_distance;

  for (const frontier_t & f : frontiers) {
    geometry_msgs::msg::Pose goal = goal_pose_from_frontier(f);

    // A goal is valid if it is far enough from any already attempted goal
    bool valid = true;
    for (const auto & attempted_goal : m_attempted_goals) {
      if (wombat_core::points_squared_distance_2d(goal.position, attempted_goal.position) <= min_dist_squared) {
        valid = false;
        break;
      }
    }
    // If we found a valid goal select this as exploration target and terminate the loop
    if (valid) {
      m_target_frontier = f;
      return goal;
    }
  }

  return std::nullopt;
}

geometry_msgs::msg::Pose
FrontiersNavigationNode::goal_pose_from_frontier(const frontier_t & frontier)
{
  geometry_msgs::msg::Pose goal;

  // The location of the goal is the one of the frontier point closest to the frontier centroid
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & point : frontier.points) {
    double distance = wombat_core::points_squared_distance_2d(point, frontier.centroid);
    if (distance < min_distance) {
      min_distance = distance;
      goal.position = point;
    }
  }

  // TODO: compute a real orientation instead of using default 0.0
  goal.orientation = wombat_core::quaternion_from_rpy(0.0, 0.0, 0.0);

  return goal;
}

void FrontiersNavigationNode::map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  m_occupancy_grid = msg;
}

void FrontiersNavigationNode::visualize_frontiers(
  const geometry_msgs::msg::Pose & goal,
  const std::vector<frontier_t> & frontiers)
{
  // Visualize frontiers as point markers.
  // The color of the frontier indicates its score: red is higher score, blue is lower score
  const float color_scale_factor = 1.0f / static_cast<float>(frontiers.size());

  visualization_msgs::msg::MarkerArray markers_msg;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = m_global_frame;
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.type = visualization_msgs::msg::Marker::POINTS;
  m.lifetime = rclcpp::Duration::from_nanoseconds(0);
  m.frame_locked = true;
  m.color.a = 1.0f;

  // Draw each frontier with a different color shade
  m.scale.x = 0.06;
  m.scale.y = 0.06;
  m.scale.z = 0.06;
  m.color.g = 0.0f;
  for (size_t i = 0; i < frontiers.size(); i++) {
    m.color.r = 1.0f - static_cast<float>(i) * color_scale_factor;
    m.color.b = static_cast<float>(i) * color_scale_factor;
    m.id = static_cast<int>(i);
    m.points = frontiers[i].points;
    markers_msg.markers.push_back(m);
  }

  // Draw goal points bigger and in green
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.03;
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.id = static_cast<int>(markers_msg.markers.size());
  m.points = {{goal.position}};
  markers_msg.markers.push_back(m);

  // Delete all old markers that have not been overwritten
  for (size_t i = markers_msg.markers.size(); i < m_prev_num_markers; i++) {
    m.action = visualization_msgs::msg::Marker::DELETE;
    m.id = static_cast<int>(i);
    markers_msg.markers.push_back(m);
  }
  m_prev_num_markers = markers_msg.markers.size();

  m_markers_pub->publish(markers_msg);
}

}  // namespace wombat_strategy
