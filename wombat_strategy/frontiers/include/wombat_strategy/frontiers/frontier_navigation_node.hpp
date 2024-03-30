// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wombat_msgs/action/frontier_navigation.hpp"
#include "wombat_strategy/frontiers/frontier_detector.hpp"
#include "wombat_strategy/frontiers/navigation_client.hpp"

namespace wombat_strategy
{

class FrontiersNavigationNode : public rclcpp::Node
{
public:
  /**
   * @brief Node constructor, initializes ROS 2 entities
   * @param options
   */
  explicit FrontiersNavigationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~FrontiersNavigationNode();

private:
  using FrontierAction = wombat_msgs::action::FrontierNavigation;
  using FrontierActionServer = nav2_util::SimpleActionServer<FrontierAction>;

  enum class StrategyState
  {
    NONE,
    START_NAVIGATION,
    NAVIGATE_TO_FRONTIER,
    DONE,
  };

  /** @brief FrontierActionServer callback, it performs the whole exploration task */
  void explore();


  bool handle_cancellation(const rclcpp::Time & start_time);

  bool start_navigation();

  bool handle_navigation();

  /**
   * @brief Select a goal location among a set of frontiers
   * @param frontiers set of frontiers among which to select the goal
   * @return std::optional<geometry_msgs::msg::Pose> goal location
   */
  std::optional<geometry_msgs::msg::Pose>
  select_navigation_goal(const std::vector<frontier_t> & frontiers);

  /**
   * @brief Select a goal location from a single frontier
   * @param frontier frontier where we want to drive
   * @return geometry_msgs::msg::Pose goal to explore this frontier
   */
  geometry_msgs::msg::Pose
  goal_pose_from_frontier(const frontier_t & frontier);

  /**
   * @brief Process an incoming map message
   * @param msg the latest map message
   */
  void map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  /**
   * @brief Publishes visualizable information about the current drive goal and frontiers
   * @param goal the goal we are driving towards
   * @param frontiers the list of active frontiers
   */
  void visualize_frontiers(const geometry_msgs::msg::Pose & goal, const std::vector<frontier_t> & frontiers);

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr m_occupancy_grid;
  FrontierDetector m_detector;

  std::shared_ptr<FrontierActionServer> m_explore_server;
  std::unique_ptr<NavigationClient> m_navigation_client;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_subscription;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_markers_pub;

  frontier_t m_target_frontier;
  geometry_msgs::msg::Pose m_curr_navigation_goal;
  std::vector<geometry_msgs::msg::Pose> m_attempted_goals;
  double m_exploration_rate {0.0};
  double m_min_attempted_goals_distance {0.0};

  std::string m_global_frame {};
  std::string m_robot_frame {};

  StrategyState m_strategy_state {StrategyState::NONE};

  std::shared_ptr<tf2_ros::Buffer> m_tf;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  size_t m_prev_num_markers {0};
};

}  // namespace wombat_strategy
