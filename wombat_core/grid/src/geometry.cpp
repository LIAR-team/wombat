// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <algorithm>
#include <array>
#include <functional>
#include <vector>

#include "wombat_core/grid/geometry.hpp"
#include "wombat_core/grid/types.hpp"
#include "wombat_core/math/utils.hpp"

namespace wombat_core
{

bool point_in_segment(const std::array<grid_coord_t, 3> & points)
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

int points_orientation(const std::array<grid_coord_t, 3> & points)
{
  const int slope =
    (points[1].y() - points[0].y()) * (points[2].x() - points[1].x()) -
    (points[1].x() - points[0].x()) * (points[2].y() - points[1].y());

  return wombat_core::sign_with_zero(slope);
}

bool points_are_collinear(const std::array<grid_coord_t, 3> & points)
{
  return points_orientation(points) == 0;
}

bool point_in_polygon(
  const grid_coord_t & point,
  const std::vector<grid_coord_t> & polygon,
  bool include_all_boundary)
{
  // Details from PNPOLY W. Randolph Franklin
  // https://wrfranklin.org/Research/Short_Notes/pnpoly.html

  size_t cross = 0;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (include_all_boundary) {
      const std::array<grid_coord_t, 3> three_points {{std::cref(polygon[j]), std::cref(point), std::cref(polygon[i])}};
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

std::pair<grid_coord_t, grid_coord_t> get_bounding_box(
  const std::vector<grid_coord_t> & points)
{
  std::pair<grid_coord_t, grid_coord_t> bbox;
  bbox.first = {
    std::numeric_limits<grid_coord_t::Scalar>::max(),
    std::numeric_limits<grid_coord_t::Scalar>::max()
  };
  bbox.second = {
    std::numeric_limits<grid_coord_t::Scalar>::lowest(),
    std::numeric_limits<grid_coord_t::Scalar>::lowest()
  };

  for (const auto & pt : points) {
    bbox.first.x() = std::min(bbox.first.x(), pt.x());
    bbox.first.y() = std::min(bbox.first.y(), pt.y());
    bbox.second.x() = std::max(bbox.second.x(), pt.x());
    bbox.second.y() = std::max(bbox.second.y(), pt.y());
  }

  return bbox;
}

}  // namespace wombat_core
