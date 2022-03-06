// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include "wombat_control/control/position_controller.hpp"
#include <math.h>

PosCtrl::PosCtrl(double len, double gain_x, double gain_y)
{
  m_len = len;
  m_gain_x = gain_x;
  m_gain_y = gain_y;
}

VelCommands PosCtrl::input_function(double e_x, double e_y, Pose2D pose)
{
  VelCommands command;

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
