// Copyright 2024 Soragna Alberto.

/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "wombat_core/grid/submap_iterator.hpp"

namespace wombat_core
{

SubmapIterator::SubmapIterator(
  const MapMetaDataAdapter & map_info,
  const wombat_core::grid_coord_t & start,
  const wombat_core::grid_size_t & submap_size)
: m_map_info(map_info)
{
  if (!grid_coord_is_valid(start, map_info)) {
    throw std::runtime_error("Invalid submap iterator start coord");
  }
  if (submap_size.x() > map_info.grid_size.x() || submap_size.y() > map_info.grid_size.y()) {
    throw std::runtime_error("Invalid submap iterator submap size");
  }

  m_current_coord = start;
  m_submap_info = m_map_info;
  m_submap_info.grid_size = submap_size;
  m_submap_min_coord = m_current_coord;
  m_current_submap_coord = {0, 0};
  m_is_past_end = false;
}

bool SubmapIterator::operator!=(const SubmapIterator & other) const
{
  return m_current_coord.x() != other.m_current_coord.x() && m_current_coord.y() != other.m_current_coord.y();
}

const grid_coord_t & SubmapIterator::operator*() const
{
  return m_current_coord;
}

SubmapIterator & SubmapIterator::operator++()
{
  m_is_past_end = !increment_submap_coord(
    m_current_submap_coord,
    m_submap_min_coord,
    m_submap_info,
    m_current_coord);

  return *this;
}

bool SubmapIterator::is_past_end() const
{
  return m_is_past_end;
}

}  // namespace wombat_core
