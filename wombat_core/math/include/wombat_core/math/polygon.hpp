// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <vector>

#include "wombat_core/math/utils.hpp"

namespace wombat_core
{

/**
 * @brief Given 3 collinear points, checks
 * whether the middle one lies within the other two.
 * Note that this function will produce wrong results if
 * used on points that are not collinear.
 * @param points list of 3 collinear points.
 * @return true if the middle points lies within the other two, false otherwise.
 */
template<typename PointT>
bool point_in_segment(const std::array<PointT, 3> & points)
{
  const auto & segment_start = points[0];
  const auto & point = points[1];
  const auto & segment_end = points[2];

  return
    point.x() <= std::max(segment_start.x(), segment_end.x()) &&
    point.x() >= std::min(segment_start.x(), segment_end.x()) &&
    point.y() <= std::max(segment_start.y(), segment_end.y()) &&
    point.y() >= std::min(segment_start.y(), segment_end.y());
}

/**
 * @brief Given 3 ordered points, compute their orientation.
 * @param points the points to check
 * @return int 0 if the points are collinear, +1 if they are ordered clockwise,
 * -1 if they are ordered counter-clockwise.
 */
template<typename PointT>
int points_orientation(const std::array<PointT, 3> & points)
{
  const int slope =
    (points[1].y() - points[0].y()) * (points[2].x() - points[1].x()) -
    (points[1].x() - points[0].x()) * (points[2].y() - points[1].y());

  return wombat_core::sign_with_zero(slope);
}

/**
 * @brief Given 3 ordered points, determine whether they are collinear
 * @param points the points to check
 * @return true if the points are collinear
 */
template<typename PointT>
bool points_are_collinear(const std::array<PointT, 3> & points)
{
  return points_orientation(points) == 0;
}

template<typename PointT>
bool point_in_polygon(
  const PointT & point,
  const std::vector<PointT> & polygon,
  bool include_edges = false)
{
  size_t cross = 0;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (include_edges) {
      const std::array<PointT, 3> three_points {{std::cref(polygon[j]), std::cref(point), std::cref(polygon[i])}};
      if (points_are_collinear(three_points) && point_in_segment(three_points)) {
        return true;
      }
    }

    if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
      (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) /
      (polygon[j].y() - polygon[i].y()) + polygon[i].x()) )
    {
      cross++;
    }
  }
  return static_cast<bool>(cross % 2);
}

}  // namespace wombat_core
