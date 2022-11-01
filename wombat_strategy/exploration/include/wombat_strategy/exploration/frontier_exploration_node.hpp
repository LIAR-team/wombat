// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wombat_msgs/action/explore.hpp"
#include "wombat_strategy/exploration/frontier_detector.hpp"

namespace wombat_strategy
{

class FrontierExplorationNode : public rclcpp::Node
{
public:
  /** @brief Node constructor, initializes ROS 2 entities */
  explicit FrontierExplorationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ExploreAction = wombat_msgs::action::Explore;
  using ExploreActionServer = nav2_util::SimpleActionServer<ExploreAction>;

  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using NavigateActionClient = rclcpp_action::Client<NavigateAction>;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateAction>;
  using NavigateCancelResponse = NavigateAction::Impl::CancelGoalService::Response;

  /** @brief ExploreActionServer callback, it performs the whole exploration task */
  void explore();

  /** @brief Select a goal location to drive to */
  geometry_msgs::msg::Pose::UniquePtr
  select_exploration_goal(const std::vector<Frontier> & frontiers);

  /** @brief Select a goal pose suitable to explore a given frontier */
  geometry_msgs::msg::Pose
  goal_pose_from_frontier(const Frontier & frontier);

  /** @brief Send a NavigateToPose action goal to drive towards chosen goal */
  void drive_to_pose(const geometry_msgs::msg::Pose & goal);

  /** @brief Navigate action server callback to be notified about whether a goal is accepted or not */
  void navigate_goal_response_callback(const NavigateGoalHandle::SharedPtr & goal_handle);

  /** @brief Navigate action server callback to be notified about navigation result */
  void navigate_result_callback(const NavigateGoalHandle::WrappedResult & result);

  /** @brief Message callback for occupancy grid topic */
  void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /** @brief Helper function that publish the frontiers as a MarkerArray message */
  void visualize_frontiers(const geometry_msgs::msg::Pose & goal, const std::vector<Frontier> & frontiers);

  std::shared_ptr<nav2_costmap_2d::Costmap2D> m_costmap;
  FrontierDetector m_detector;

  std::shared_ptr<ExploreActionServer> m_explore_server;
  std::shared_ptr<NavigateActionClient> m_navigate_client;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_subscription;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_markers_pub;

  NavigateGoalHandle::SharedPtr m_navigation_goal_handle;
  geometry_msgs::msg::Pose m_current_goal;
  Frontier m_target_frontier;
  std::vector<geometry_msgs::msg::Pose> m_attempted_goals;
  double m_exploration_rate;
  double m_min_attempted_goals_distance;

  std::string m_global_frame;
  std::string m_robot_frame;

  std::shared_ptr<tf2_ros::Buffer> m_tf;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  tf2::Stamped<Eigen::Affine3d> m_last_progress_pose;
  rclcpp::Time m_last_progress_time;
  double m_linear_progress_dist;
  double m_angular_progress_dist;
  rclcpp::Duration m_no_progress_timeout;
  bool m_publish_frontiers;
  size_t m_prev_num_frontiers;
};

}  // namespace wombat_strategy
