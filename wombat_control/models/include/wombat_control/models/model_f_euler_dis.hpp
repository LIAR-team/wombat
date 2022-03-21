// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#pragma once

#include "wombat_control/data_types/pose2d.hpp"

class KinModel
{
public:
  // v [m/s], omega [rad/s], delta_t [s]
  Pose2D integration(double v, double omega, double delta_t);
  explicit KinModel(Pose2D initial_pose);

private:
  Pose2D m_current_pose;
};
