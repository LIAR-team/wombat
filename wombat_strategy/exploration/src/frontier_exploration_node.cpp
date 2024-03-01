// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "wombat_strategy/exploration/frontier_exploration_node.hpp"

#include "tf2_ros/create_timer_ros.h"

#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "wombat_core/math/geometry_point.hpp"

namespace wombat_strategy
{

FrontierExplorationNode::FrontierExplorationNode(const rclcpp::NodeOptions & options)
: Node("frontier_exploration", options),
  m_no_progress_timeout(rclcpp::Duration::from_nanoseconds(0))
{
  // Node parameters
  std::string map_topic;
  bool search_only_free_space = false;
  int min_unknown_neighbors = 0;
  int min_free_neighbors = 0;
  int max_occupied_neighbors = 0;
  int min_frontier_size = 0;
  double frontier_size_scaling_factor = 0.0;
  int no_progress_seconds = 0;
  this->get_parameter_or("global_frame", m_global_frame, std::string("map"));
  this->get_parameter_or("robot_frame", m_robot_frame, std::string("base_link"));
  this->get_parameter_or("map_topic", map_topic, std::string("map"));
  this->get_parameter_or("exploration_rate", m_exploration_rate, 5.0);
  this->get_parameter_or("min_attempted_goals_distance", m_min_attempted_goals_distance, 0.2);
  this->get_parameter_or("search_only_free_space", search_only_free_space, true);
  this->get_parameter_or("min_unknown_neighbors", min_unknown_neighbors, 1);
  this->get_parameter_or("min_free_neighbors", min_free_neighbors, 3);
  this->get_parameter_or("max_occupied_neighbors", max_occupied_neighbors, 1);
  this->get_parameter_or("min_frontier_size", min_frontier_size, 10);
  this->get_parameter_or("frontier_size_scaling_factor", frontier_size_scaling_factor, 0.0);
  this->get_parameter_or("linear_progress_dist", m_linear_progress_dist, 0.15);
  this->get_parameter_or("angular_progress_dist", m_angular_progress_dist, M_PI / 9.0);
  this->get_parameter_or("no_progress_seconds", no_progress_seconds, 10);
  m_no_progress_timeout = rclcpp::Duration(no_progress_seconds, 0);
  this->get_parameter_or("publish_frontiers", m_publish_frontiers, true);

  m_map_subscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic,
    10,
    std::bind(&FrontierExplorationNode::map_callback, this, std::placeholders::_1));

  m_prev_num_frontiers = 0;
  if (m_publish_frontiers) {
    m_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 10);
  }

  // Exploration server, used by user or application to start an exploration task
  m_explore_server = std::make_shared<ExploreActionServer>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "explore",
    std::bind(&FrontierExplorationNode::explore, this));
  m_explore_server->activate();

  // Navigation client, used to drive the robot during the exploration
  m_navigate_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "navigate_to_pose");

  // TF tools, used to get up to date robot pose
  m_tf = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  m_tf->setCreateTimerInterface(timer_interface);
  m_tf->setUsingDedicatedThread(true);
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf, this, false);

  FrontierDetector::params_t params;
  params.search_only_free_space = search_only_free_space;
  params.min_unknown_neighbors = min_unknown_neighbors;
  params.min_free_neighbors = min_free_neighbors;
  params.max_occupied_neighbors = max_occupied_neighbors;
  params.min_frontier_size = min_frontier_size;
  params.frontier_size_scaling_factor = frontier_size_scaling_factor;
  m_detector = FrontierDetector(params);

  RCLCPP_INFO(this->get_logger(), "Node created");
}

void FrontierExplorationNode::explore()
{
  auto goal = m_explore_server->get_current_goal();
  RCLCPP_INFO(this->get_logger(), "Starting exploration for %d frontiers!", goal->num_frontiers);

  auto start_time = this->now();
  auto result = std::make_shared<typename ExploreAction::Result>();

  int remaining_frontiers = goal->num_frontiers;
  rclcpp::Rate loop_rate(m_exploration_rate);
  while (rclcpp::ok()) {
    if (m_explore_server->is_cancel_requested()) {
      RCLCPP_INFO(this->get_logger(), "Canceling exploration");
      result->total_elapsed_time = this->now() - start_time;
      m_explore_server->terminate_all(result);

      // Cancel current navigation goal when exiting
      if (m_navigation_goal_handle) {
        m_navigate_client->async_cancel_goal(m_navigation_goal_handle);
      }
      return;
    }

    // A map is required for exploring, keep waiting until it is available
    if (!m_occupancy_grid) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *(this->get_clock()), 1000,
        "No map available, can't explore");
      loop_rate.sleep();
      continue;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    tf2::Stamped<Eigen::Affine3d> current_tf;
    nav2_util::getCurrentPose(current_pose, *m_tf, m_global_frame, m_robot_frame);
    tf2::fromMsg(current_pose, current_tf);
    // If we have an active navigation goal, leave the control to navigation stack
    if (m_navigation_goal_handle) {
      // Check if is it still worth exploring target frontier
      constexpr bool TRUSTED = false;
      bool valid = m_detector.frontier_is_valid(
        m_target_frontier,
        m_occupancy_grid,
        wombat_core::MapMetaDataAdapter(m_occupancy_grid->info),
        TRUSTED);
      if (valid) {
        // Now check if it is making progress
        Eigen::Affine3d last_progress_pose_inv = m_last_progress_pose.inverse();
        auto travel = last_progress_pose_inv * current_tf;
        auto linear_dist = travel.translation().norm();
        auto euler = travel.rotation().eulerAngles(0, 1, 2);
        auto angular_dist = std::abs(euler[2]);
        auto time_since_progress = this->now() - m_last_progress_time;
        if (linear_dist > m_linear_progress_dist || angular_dist > m_angular_progress_dist) {
          m_last_progress_pose = current_tf;
          m_last_progress_time = this->now();
          RCLCPP_DEBUG(
            this->get_logger(),
            "Made sufficient progress linear %f, angular %f in set time, reset progress pose",
            linear_dist,
            angular_dist);
        } else if (time_since_progress > m_no_progress_timeout) {
          RCLCPP_INFO(
            this->get_logger(),
            "Failed to make sufficient progress linear %f, angular %f in set time %f, cancelling navigation",
            linear_dist,
            angular_dist,
            time_since_progress.nanoseconds() / 1e9);
          m_navigate_client->async_cancel_goal(m_navigation_goal_handle);
          m_navigation_goal_handle = nullptr;
          m_attempted_goals.push_back(m_current_goal);
        } else {
          RCLCPP_DEBUG(
            this->get_logger(),
            "Driving towards valid frontier, progress linear %f, angular %f, time %f nothing to do here",
            linear_dist,
            angular_dist,
            time_since_progress.nanoseconds() / 1e9);
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "Target frontier is not valid anymore, cancelling navigation");
        m_navigate_client->async_cancel_goal(m_navigation_goal_handle);
        m_navigation_goal_handle = nullptr;
      }
      loop_rate.sleep();
      continue;
    }

    // Get current robot pose
    RCLCPP_INFO(this->get_logger(), "Current position: %f %f", current_pose.pose.position.x, current_pose.pose.position.y);

    // Search new frontiers
    auto frontiers = m_detector.search_frontiers(m_occupancy_grid, current_pose.pose.position);
    RCLCPP_INFO(this->get_logger(), "Found %lu frontiers", frontiers.size());

    // Select exploration goal
    geometry_msgs::msg::Pose::UniquePtr exploration_goal = select_exploration_goal(frontiers);

    // Ready to select a new frontier or to terminate if all required frontiers have been explored
    if (remaining_frontiers == 0 || frontiers.empty() || !exploration_goal) {
      result->total_elapsed_time = this->now() - start_time;
      m_explore_server->succeeded_current(result);
      RCLCPP_INFO(this->get_logger(), "Completed exploration!");
      return;
    }

    // Start driving towards selected target pose
    drive_to_pose(*exploration_goal);
    m_last_progress_pose = current_tf;
    m_last_progress_time = this->now();

    // Debugging visualization tools
    if (m_publish_frontiers) {
      visualize_frontiers(*exploration_goal, frontiers);
    }

    remaining_frontiers--;
    loop_rate.sleep();
  }
}

geometry_msgs::msg::Pose::UniquePtr
FrontierExplorationNode::select_exploration_goal(
  const std::vector<frontier_t> & frontiers)
{
  // Frontiers are already sorted according to score.
  // We compute a goal location from each sorted frontiers and we select the first valid one.

  const double min_dist_squared = m_min_attempted_goals_distance * m_min_attempted_goals_distance;
  // TODO: this should be optional, not a pointer
  geometry_msgs::msg::Pose::UniquePtr exploration_goal;

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
      exploration_goal = std::make_unique<geometry_msgs::msg::Pose>(goal);
      break;
    }
  }

  return exploration_goal;
}

geometry_msgs::msg::Pose
FrontierExplorationNode::goal_pose_from_frontier(const frontier_t & frontier)
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
  // Even if laser is 360, this may result in robot reaching goal and unnecessarily turning to this orientation
  goal.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);

  return goal;
}

void FrontierExplorationNode::drive_to_pose(const geometry_msgs::msg::Pose & goal)
{
  // Store current goal pose
  m_current_goal = goal;

  nav2_msgs::action::NavigateToPose::Goal action_goal;
  action_goal.pose.header.stamp = this->now();
  action_goal.pose.header.frame_id = m_global_frame;
  action_goal.pose.pose = m_current_goal;

  // Set callbacks to be notified about this asynchronous goal
  auto send_goal_options = NavigateActionClient::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const NavigateGoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Navigation goal was rejected by the server");
        return;
      }
      m_navigation_goal_handle = goal_handle;
    };
  send_goal_options.result_callback =
    std::bind(&FrontierExplorationNode::navigate_result_callback, this, std::placeholders::_1);

  // Send the goal request
  RCLCPP_INFO(
    this->get_logger(),
    "Navigation goal: %f %f",
    action_goal.pose.pose.position.x,
    action_goal.pose.pose.position.y);
  m_navigate_client->async_send_goal(action_goal, send_goal_options);
}

void FrontierExplorationNode::navigate_result_callback(const NavigateGoalHandle::WrappedResult & result)
{
  // Regardless of the result, the navigation towards the current goal is finished
  m_navigation_goal_handle = nullptr;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Navigation to pose succeeded");
      // Sometimes robot gets close enough to achieve goal, but frontier still
      // exists and robot continues to attempt to navigate there as next goal
      m_attempted_goals.push_back(m_current_goal);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      // This should happens when the navigation stack considers current goal unreachable
      RCLCPP_ERROR(this->get_logger(), "Navigation to pose aborted");
      m_attempted_goals.push_back(m_current_goal);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // This should happen when this node cancels the currently active goal
      RCLCPP_INFO(this->get_logger(), "Navigation to pose cancelled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Navigate server returned unknown result code");
      break;
  }
}

void FrontierExplorationNode::map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  m_occupancy_grid = msg;
}

void FrontierExplorationNode::visualize_frontiers(
  const geometry_msgs::msg::Pose & goal,
  const std::vector<frontier_t> & frontiers)
{
  RCLCPP_DEBUG(this->get_logger(), "Visualizing frontiers");

  // Visualize frontiers as point markers.
  // The color of the frontier indicates its score: red is higher score, blue is lower score
  const float color_scale_factor = 1.0f / static_cast<float>(frontiers.size());

  visualization_msgs::msg::MarkerArray markers_msg;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = m_global_frame;
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 0.03;
  m.scale.y = 0.03;
  m.scale.z = 0.03;
  m.color.g = 0.0f;
  m.color.a = 1.0f;
  m.type = visualization_msgs::msg::Marker::POINTS;
  m.lifetime = rclcpp::Duration::from_nanoseconds(0);
  m.frame_locked = true;

  for (size_t i = 0; i < frontiers.size(); i++) {
    m.color.r = 1.0f - static_cast<float>(i) * color_scale_factor;
    m.color.b = static_cast<float>(i) * color_scale_factor;
    m.id = static_cast<int>(i);
    m.points = frontiers[i].points;
    markers_msg.markers.push_back(m);
  }
  // Draw goal points in green
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.03;
  m.id = static_cast<int>(frontiers.size());
  m.points = {{goal.position}};
  markers_msg.markers.push_back(m);
  // Frontiers + goal marker
  unsigned int current_markers = frontiers.size() + 1;

  // Old markers that are not not going to be overwritten have to be deleted
  if (m_prev_num_frontiers > frontiers.size()) {
    m.action = visualization_msgs::msg::Marker::DELETE;
    for (size_t i = current_markers; i < m_prev_num_frontiers; i++) {
      m.id = static_cast<int>(i);
      markers_msg.markers.push_back(m);
    }
  }
  m_prev_num_frontiers = current_markers;

  m_markers_pub->publish(markers_msg);
}

}  // namespace wombat_strategy
