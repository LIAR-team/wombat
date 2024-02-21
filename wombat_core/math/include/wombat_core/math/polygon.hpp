// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <vector>

namespace wombat_core
{

template<typename PointT>
bool point_in_polygon(const PointT & point, const std::vector<PointT> & polygon)
{
  size_t cross = 0;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
      (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) /
      (polygon[j].y() - polygon[i].y()) + polygon[i].x()) )
    {
      cross++;
    }
  }
  return static_cast<bool>(cross % 2);
}

}
