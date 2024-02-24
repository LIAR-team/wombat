// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <memory>

#include "wombat_core/grid/polygon_iterator.hpp"
#include "wombat_core/math/polygon.hpp"

namespace wombat_core
{

static std::pair<grid_coord_t, grid_coord_t> get_bounding_box(
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
  assert(!points.empty());
  for (const auto & pt : points) {
    bbox.first.x() = std::min(bbox.first.x(), pt.x());
    bbox.first.y() = std::min(bbox.first.y(), pt.y());
    bbox.second.x() = std::max(bbox.second.x(), pt.x());
    bbox.second.y() = std::max(bbox.second.y(), pt.y());
  }

  return bbox;
}

PolygonIterator::PolygonIterator(
  const MapMetaDataAdapter & map_info,
  const std::vector<grid_coord_t> & polygon)
: m_polygon(polygon)
{
  auto bbox = get_bounding_box(polygon);
  assert(grid_coord_is_valid(bbox.first, map_info));
  assert(grid_coord_is_valid(bbox.second, map_info));

  auto bbox_size = get_grid_size_from_corners(bbox.first, bbox.second);

  m_internal_iterator = std::make_unique<SubmapIterator>(map_info, bbox.first, bbox_size);
  assert(!m_internal_iterator->is_past_end());
  if (!isInside()) {++(*this);}
}

bool PolygonIterator::operator!=(const PolygonIterator & other) const
{
  return m_internal_iterator != other.m_internal_iterator;
}

const grid_coord_t & PolygonIterator::operator*() const
{
  return *(*m_internal_iterator);
}

PolygonIterator & PolygonIterator::operator++()
{
  ++(*m_internal_iterator);
  if (m_internal_iterator->is_past_end()) {
    return *this;
  }

  for (; !m_internal_iterator->is_past_end(); ++(*m_internal_iterator)) {
    if (isInside()) {
      break;
    }
  }

  return *this;
}

bool PolygonIterator::is_past_end() const
{
  return m_internal_iterator->is_past_end();
}

bool PolygonIterator::isInside() const
{
  return point_in_polygon(*(*m_internal_iterator), m_polygon);
}

}  // namespace wombat_core
