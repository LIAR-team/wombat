// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#pragma once

#include "wombat_control/data_types/pose2d.hpp"
#include "wombat_control/data_types/vel_commands.hpp"

class PosCtrl
{
public:
  PosCtrl(double len, double gain_x, double gain_y);
  VelCommands input_function(double e_x, double e_y, Pose2D pose) const;

private:
  double m_len;
  double m_gain_x;
  double m_gain_y;
};
