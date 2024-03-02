// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <vector>

namespace wombat_core
{

/**
 * @brief Given 3 collinear points, checks
 * whether the middle one lies within the other two.
 * Note that this function will produce wrong results if
 * used on points that are not collinear.
 */
template<typename PointT>
bool point_in_segment(
  const PointT & segment_start,
  const PointT & point,
  const PointT & segment_end)
{
  return
    point.x() <= std::max(segment_start.x(), segment_end.x()) &&
    point.x() >= std::min(segment_start.x(), segment_end.x()) &&
    point.y() <= std::max(segment_start.y(), segment_end.y()) &&
    point.y() >= std::min(segment_start.y(), segment_end.y());
}

template<typename PointT>
int points_orientation(const PointT  & p, const PointT  & q, const PointT  & r)
{
    int val = (q.y() - p.y()) * (r.x() - q.x()) -
              (q.x() - p.x()) * (r.y() - q.y());

    // colinear
    if (val == 0) {
      return 0;
    }
    return val > 0 ? 1 : 2; // clock or counterclock wise
}

template<typename PointT>
bool point_in_polygon(
  const PointT & point,
  const std::vector<PointT> & polygon,
  bool include_edges = false)
{
  size_t cross = 0;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (include_edges &&
      points_orientation(polygon[j], point, polygon[i]) == 0 && point_in_segment(polygon[j], point, polygon[i]))
    {
      return true;
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
