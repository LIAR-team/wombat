// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#pragma once

#include "wombat_control/data_types/vel_commands.hpp"
#include "wombat_msgs/pose2d.hpp"

class KinModel
{
public:
  explicit KinModel(const Pose2D & initial_pose);

  // v [m/s], omega [rad/s], delta_t [s]
  Pose2D integration(const VelCommands & vel_commands, double delta_t);

private:
  Pose2D m_current_pose;
};
