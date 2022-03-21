// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include "wombat_control/models/model_f_euler_dis.hpp"

#include <math.h>

KinModel::KinModel(Pose2D initial_pose) {m_current_pose = initial_pose;}

Pose2D KinModel::integration(double v, double omega, double delta_t)
{
  Pose2D next_pose;

  // Integration
  next_pose.x = m_current_pose.x + delta_t * (v * std::cos(m_current_pose.theta));
  next_pose.y = m_current_pose.y + delta_t * (v * std::sin(m_current_pose.theta));
  next_pose.theta = m_current_pose.theta + delta_t * (omega);

  // Update current pose
  m_current_pose = next_pose;

  return next_pose;
}
