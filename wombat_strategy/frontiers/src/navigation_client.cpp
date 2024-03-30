// Copyright 2021-2024 Soragna Alberto.

#include "wombat_strategy/frontiers/navigation_client.hpp"

#include "geometry_msgs/msg/point.hpp"

namespace wombat_strategy
{

NavigationClient::NavigationClient(
  std::shared_ptr<wombat_core::NodeInterfaces> node_interfaces,
  const rclcpp::Duration & no_progress_timeout)
: m_clock(node_interfaces->get_node_clock_interface()->get_clock()),
m_logger(node_interfaces->get_node_logging_interface()->get_logger())
{
  (void)no_progress_timeout;

  // "drive to pose" action client
  m_navigate_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_interfaces->get_node_base_interface(),
    node_interfaces->get_node_graph_interface(),
    node_interfaces->get_node_logging_interface(),
    node_interfaces->get_node_waitables_interface(),
    "navigate_to_pose");

  RCLCPP_INFO(m_logger, "Navigation client created");
}

NavigationClient::~NavigationClient()
{
  cancel_navigate_to_pose(rclcpp::Duration::from_seconds(0.0));
}

void NavigationClient::start_navigate_to_pose(
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  if (m_future_goal_handle.valid()) {
    throw std::runtime_error("Calling start while a goal handle is already present");
  }

  // Update current navigator goal pose
  m_navigate_to_pose_goal.pose = goal_pose;

  m_future_goal_handle = m_navigate_client->async_send_goal(m_navigate_to_pose_goal);

  // Send the goal request
  RCLCPP_INFO(
    m_logger,
    "Navigate to pose goal: %f %f",
    m_navigate_to_pose_goal.pose.pose.position.x,
    m_navigate_to_pose_goal.pose.pose.position.y);
}

NavigationClient::ResultWithStatus
NavigationClient::handle_navigate_to_pose(
  const geometry_msgs::msg::PoseStamped & current_pose)
{
  (void)current_pose;

  if (!m_future_goal_handle.valid()) {
    throw std::runtime_error("Calling handle without request sent");
  }

  // Check if we have navigation goal handle
  if (!m_navigation_goal_handle) {
    auto status = m_future_goal_handle.wait_for(std::chrono::seconds(0));
    if (status != std::future_status::ready) {
      return ResultWithStatus(Status::RUNNING);
    }
    m_navigation_goal_handle = m_future_goal_handle.get();
    m_future_result = m_navigate_client->async_get_result(m_navigation_goal_handle);
  }

  if (!m_future_result.valid()) {
    throw std::runtime_error("Calling handle without result valid");
  }

  // Check if action is done
  auto status = m_future_result.wait_for(std::chrono::seconds(0));
  if (status == std::future_status::ready) {
    auto wrapped_result = m_future_result.get();
    clear();
    return ResultWithStatus(Status::DONE, wrapped_result);
  }

  if (!is_making_progress()) {
    this->cancel_navigate_to_pose(rclcpp::Duration::from_seconds(0.0));
    return ResultWithStatus(Status::CANCELED);
  }

  return ResultWithStatus(Status::RUNNING);
}

void NavigationClient::cancel_navigate_to_pose(const rclcpp::Duration & goal_handle_timeout)
{
  if (!m_future_goal_handle.valid()) {
    RCLCPP_INFO(m_logger, "Navigation client can't cancel if no goal was sent");
    return;
  }

  if (!rclcpp::ok()) {
    RCLCPP_INFO(m_logger, "Navigation client can't cancel goal without ros context");
    return;
  }

  if (!m_navigation_goal_handle) {
    auto status = m_future_goal_handle.wait_for(
      goal_handle_timeout.to_chrono<std::chrono::milliseconds>());
    if (status != std::future_status::ready) {
      clear();
      RCLCPP_INFO(m_logger, "Navigation client can't cancel goal without a goal handle");
      return;
    }
    m_navigation_goal_handle = m_future_goal_handle.get();
  }

  m_navigate_client->async_cancel_goal(m_navigation_goal_handle);
  clear();
}

bool NavigationClient::is_making_progress()
{
  return true;
}

void NavigationClient::clear()
{
  m_future_goal_handle = std::shared_future<GoalHandle::SharedPtr>();
  m_future_result = std::shared_future<WrappedResult>();
  m_navigation_goal_handle = nullptr;
}

}  // namespace wombat_strategy
