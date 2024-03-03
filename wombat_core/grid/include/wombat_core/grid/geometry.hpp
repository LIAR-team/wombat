// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <array>
#include <utility>
#include <vector>

#include "wombat_core/grid/types.hpp"

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
bool point_in_segment(const std::array<grid_coord_t, 3> & points);

/**
 * @brief Given 3 ordered points, compute their orientation.
 * @param points the points to check
 * @return int 0 if the points are collinear, +1 if they are ordered clockwise,
 * -1 if they are ordered counter-clockwise.
 */
int points_orientation(const std::array<grid_coord_t, 3> & points);

/**
 * @brief Given 3 ordered points, determine whether they are collinear
 * @param points the points to check
 * @return true if the points are collinear
 */
bool points_are_collinear(const std::array<grid_coord_t, 3> & points);

/**
 * @brief Checks whether a point lies inside a polygon.
 * This implementation will, by default, treat the "bottom" and "left" edges
 * as inside the polygon, while points on the other edges will be considered outside.
 * The purpose is to guarantee that if the space is partitioned into multiple polygons,
 * each point belongs to 1 and only 1 polygon.
 * @param point the point
 * @param polygon the polygon
 * @param include_all_boundary if true, points on all edges will be considered as internal.
 * This breaks the assumption that each grid point belongs to only 1 non-overlapping polygon.
 * @return true if the point is internal
 */
bool point_in_polygon(
  const grid_coord_t & point,
  const std::vector<grid_coord_t> & polygon,
  bool include_all_boundary = false);

/**
 * @brief Compute a bounding box as a pair of min/max coordinates
 * from a list of points
 * @param points the points that should be included in the box
 * @return std::pair<grid_coord_t, grid_coord_t> min and max vertices of the box
 */
std::pair<grid_coord_t, grid_coord_t> get_bounding_box(
  const std::vector<grid_coord_t> & points);

}  // namespace wombat_core
