// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include "wombat_control/control/position_controller.hpp"
#include <math.h>

PosCtrl::PosCtrl(const Params & params)
{
  m_len = params.len;
  m_gain_x = params.gain_x;
  m_gain_y = params.gain_y;
}

VelCommands PosCtrl::input_function(const Eigen::Vector2d & error, const Pose2D & pose) const
{
  VelCommands command;

  double e_x = error[0];
  double e_y = error[1];

  double stheta = std::sin(pose.theta);
  double ctheta = std::cos(pose.theta);

  double fb_x = -m_gain_x * e_x;
  double fb_y = -m_gain_y * e_y;

  command.v = fb_x * ctheta + fb_y * stheta;
  command.omega = (1 / m_len) * ((-fb_x * stheta) + (fb_y * ctheta));

  if (std::abs(command.v) <= 1e-5) {
    command.v = 0;
  }

  if (std::abs(command.omega) <= 1e-5) {
    command.omega = 0;
  }

  return command;
}
