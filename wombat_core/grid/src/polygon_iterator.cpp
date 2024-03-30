// Copyright 2024 Soragna Alberto.

/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <memory>

#include "wombat_core/grid/geometry.hpp"
#include "wombat_core/grid/polygon_iterator.hpp"

namespace wombat_core
{

PolygonIterator::PolygonIterator(
  const MapMetaDataAdapter & map_info,
  const std::vector<grid_coord_t> & polygon,
  bool include_boundary)
: m_polygon(polygon), m_include_boundary(include_boundary)
{
  auto bbox = get_bounding_box(polygon);
  if (!grid_coord_is_valid(bbox.first, map_info)) {
    throw std::runtime_error("Invalid bounding box min coord");
  }
  if (!grid_coord_is_valid(bbox.second, map_info)) {
    throw std::runtime_error("Invalid bounding box max coord");
  }

  auto bbox_size = get_grid_size_from_corners(bbox.first, bbox.second);

  m_internal_iterator = std::make_unique<SubmapIterator>(map_info, bbox.first, bbox_size);
  if (m_internal_iterator->is_past_end()) {
    throw std::runtime_error("Failed to setup polygon internal iterator");
  }

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
  return point_in_polygon(
    *(*m_internal_iterator),
    m_polygon,
    m_include_boundary);
}

}  // namespace wombat_core
