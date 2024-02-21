// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

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

/*!
 * Increases the index by one to iterate through the cells of a submap.
 * Increments either to the neighboring index to the right or to
 * the start of the lower row. Returns false if end of iterations are reached.
 *
 * Note: This function does not check if submap actually fits to the map. This needs
 * to be checked before separately.
 *
 * @param[in/out] submap_coord the index in the submap that is incremented.
 * @param[out] index the index in the map that is incremented (corrected for the circular buffer).
 * @param[in] submapTopLefIndex the top left index of the submap.
 * @param[in] submapBufferSize the submap buffer size.
 * @param[in] bufferStartIndex the map buffer start index.
 * @return true if successfully incremented indices, false if end of iteration limits are reached.
 */
static bool increment_submap_coord(
  grid_coord_t & submap_coord,
  const grid_coord_t & submap_top_left_coord,
  const MapMetaDataAdapter & submap_info,
  grid_coord_t & map_coord)
{
  // Copy the data first, only copy it back if everything is within range.
  auto tmp_submap_coord = submap_coord;

  // Increment submap index.
  if (tmp_submap_coord.x() < submap_info.grid_size.x() - 1) {
    // Same row.
    tmp_submap_coord.x()++;
  } else {
    // Next row.
    tmp_submap_coord.y()++;
    tmp_submap_coord.x() = 0;
  }

  // End of iterations reached.
  if (!grid_coord_is_valid(tmp_submap_coord, submap_info)) {
    return false;
  }

  // Copy data back.
  map_coord = submap_top_left_coord + tmp_submap_coord;
  submap_coord = tmp_submap_coord;
  return true;
}

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
  m_submap_top_left_coord = m_current_coord;
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
    m_submap_top_left_coord,
    m_submap_info,
    m_current_coord);

  return *this;
}

bool SubmapIterator::is_past_end() const
{
  return m_is_past_end;
}

}  // namespace wombat_core
