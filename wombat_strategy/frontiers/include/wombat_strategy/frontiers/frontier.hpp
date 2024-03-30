// Copyright 2021-2022 Soragna Alberto.

#pragma once

#include <vector>

#include <geometry_msgs/msg/point.hpp>

namespace wombat_strategy
{

struct frontier_t
{
  /** @brief Points belonging to frontier in map reference frame */
  std::vector<geometry_msgs::msg::Point> points {};
  /** @brief Center of mass of @ref points */
  geometry_msgs::msg::Point centroid;
  /** @brief Score assigned by the frontier detector to this frontier */
  double score {0.0};

  bool operator>(const frontier_t & other) const
  {
    return score > other.score;
  }
};

}  // namespace wombat_strategy
