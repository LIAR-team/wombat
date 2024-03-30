// Copyright 2024 Soragna Alberto.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "wombat_core/ros2/node_interfaces.hpp"

namespace wombat_strategy
{

class NavigationClient
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using ActionClient = rclcpp_action::Client<Action>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using WrappedResult = GoalHandle::WrappedResult;

  enum class Status
  {
    IDLE,
    RUNNING,
    DONE,
    CANCELED,
  };

  struct result_with_status_t
  {
    result_with_status_t() = default;
    explicit result_with_status_t(Status s)
    : status(s)
    {}

    result_with_status_t(Status s, const WrappedResult & res)
    : status(s), wrapped_result(res)
    {}

    Status status;
    std::optional<WrappedResult> wrapped_result;
  };

  NavigationClient(
    std::shared_ptr<wombat_core::NodeInterfaces> node_interfaces,
    const rclcpp::Duration & no_progress_timeout);

  ~NavigationClient();

  void start_navigate_to_pose(const geometry_msgs::msg::PoseStamped & goal_pose);

  result_with_status_t handle_navigate_to_pose(const geometry_msgs::msg::PoseStamped & current_pose);

  void cancel_navigate_to_pose(const rclcpp::Duration & goal_handle_timeout);

private:
  bool is_making_progress();

  void clear();

  std::shared_ptr<ActionClient> m_navigate_client;

  Action::Goal m_navigate_to_pose_goal;
  std::shared_future<GoalHandle::SharedPtr> m_future_goal_handle;
  GoalHandle::SharedPtr m_navigation_goal_handle;
  std::shared_future<WrappedResult> m_future_result;

  rclcpp::Clock::SharedPtr m_clock;
  rclcpp::Logger m_logger;
};

}  // namespace wombat_strategy
