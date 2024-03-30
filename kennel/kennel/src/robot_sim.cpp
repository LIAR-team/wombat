// Copyright 2024 Soragna Alberto.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kennel/mobile_base/mobile_base.hpp"
#include "kennel/robot_sim.hpp"
#include "wombat_core/ros2/parameters.hpp"

namespace kennel
{

RobotSim::RobotSim(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_sim", options)
{
  m_mobile_base = std::make_shared<MobileBase>(this);
  m_sensors_manager = std::make_shared<SensorsManager>(this);

  // TODO: is wall time ok here? Should we use simulated time?
  m_mobile_base_timer = this->create_wall_timer(
    m_mobile_base->get_update_period(),
    [this]() {
      m_mobile_base->mobile_base_update();
    });
  m_sensors_timer = this->create_wall_timer(
    m_sensors_manager->get_update_period(),
    [this]() {
      auto gt_data = m_mobile_base->get_ground_truth_data();
      m_sensors_manager->sensors_update(gt_data);
    });

  RCLCPP_INFO(this->get_logger(), "Robot simulation constructed");
}

}  // namespace kennel
