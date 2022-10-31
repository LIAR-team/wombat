// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#pragma once

#include "Eigen/Core"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

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

  geometry_msgs::msg::Twist input_function(
    const Eigen::Vector2d & error,
    const geometry_msgs::msg::Pose & pose) const;

private:
  double m_len;
  double m_gain_x;
  double m_gain_y;
};
