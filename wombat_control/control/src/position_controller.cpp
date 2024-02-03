// Copyright 2021-2022 Azzollini Ilario, Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <math.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "wombat_control/control/position_controller.hpp"
#include "wombat_core/math/utils.hpp"

PositionController::PositionController(const params_t & params)
{
  m_len = params.len;
  m_gain_x = params.gain_x;
  m_gain_y = params.gain_y;
}

geometry_msgs::msg::Twist PositionController::input_function(
  const Eigen::Vector2d & error,
  const geometry_msgs::msg::Pose & pose) const
{
  double theta = tf2::getYaw(pose.orientation);

  double stheta = std::sin(theta);
  double ctheta = std::cos(theta);

  double fb_x = -m_gain_x * error.x();
  double fb_y = -m_gain_y * error.y();

  double v = fb_x * ctheta + fb_y * stheta;
  double omega = (1 / m_len) * ((-fb_x * stheta) + (fb_y * ctheta));

  v = wombat_core::clamp_to_zero(v, 1e-5);
  omega = wombat_core::clamp_to_zero(omega, 1e-5);

  geometry_msgs::msg::Twist command;
  command.linear.x = v;
  command.angular.z = omega;

  return command;
}
