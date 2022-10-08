// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#pragma once

#include "Eigen/Core"

#include "wombat_msgs/pose2d.hpp"
#include "wombat_control/data_types/vel_commands.hpp"

class PosCtrl
{
public:
  struct Params
  {
    double len {0.0};
    double gain_x {0.0};
    double gain_y {0.0};
  };

  explicit PosCtrl(const Params & params);

  VelCommands input_function(const Eigen::Vector2d & error, const Pose2D & pose) const;

private:
  double m_len;
  double m_gain_x;
  double m_gain_y;
};
