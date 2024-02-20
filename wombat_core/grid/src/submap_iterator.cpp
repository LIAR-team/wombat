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
  MapMetaDataAdapter map_info,
  const wombat_core::grid_coord_t & start,
  const wombat_core::grid_size_t & submap_size)
: m_map_info(std::move(map_info))
{
  m_current_coord = start;
  m_submap_info = m_map_info;
  m_submap_info.grid_size = submap_size;
  m_submap_top_left_coord = m_current_coord;
  m_current_submap_coord = {0, 0};
  isPastEnd_ = false;
}

bool SubmapIterator::operator !=(const SubmapIterator& other) const
{
  return m_current_coord.x() != other.m_current_coord.x() && m_current_coord.y() != other.m_current_coord.y();
}

const grid_coord_t & SubmapIterator::operator *() const
{
  return m_current_coord;
}

SubmapIterator & SubmapIterator::operator ++()
{
  isPastEnd_ = !increment_index_for_submap(
    m_current_submap_coord,
    m_current_coord,
    m_submap_top_left_coord,
    m_submap_info);

  return *this;
}

bool SubmapIterator::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace grid_map */
