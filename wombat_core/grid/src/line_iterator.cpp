// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "wombat_core/grid/line_iterator.hpp"

namespace wombat_core
{

LineIterator::LineIterator(
  const MapMetaDataAdapter & map_info,
  const wombat_core::grid_coord_t & start,
  const wombat_core::grid_coord_t & end)
{
  (void)map_info;

  m_start_coord = start;
  m_end_coord = end;

  m_current_cell_count = 0;
  m_current_coord = start;

  if (m_end_coord.x() >= m_start_coord.x()) {
    // x-values increasing.
    m_increment1.x() = 1;
    m_increment2.x() = 1;
  } else {
    // x-values decreasing.
    m_increment1.x() = -1;
    m_increment2.x() = -1;
  }

  if (m_end_coord.y() >= m_start_coord.y()) {
    // y-values increasing.
    m_increment1.y() = 1;
    m_increment2.y() = 1;
  } else {
    // y-values decreasing.
    m_increment1.y() = -1;
    m_increment2.y() = -1;
  }

  const int dx = static_cast<int>(m_end_coord.x() - m_start_coord.x());
  const int dy = static_cast<int>(m_end_coord.y() - m_start_coord.y());
  const int abs_dx = std::abs(dx);
  const int abs_dy = std::abs(dy);

  if (abs_dx >= abs_dy) {
    // There is at least one x-value for every y-value.
    m_increment1.x() = 0;  // Do not change the x when numerator >= denominator.
    m_increment2.y() = 0;  // Do not change the y for every iteration.
    m_denominator = abs_dx;
    m_numerator = abs_dx / 2;
    m_numerator_add = abs_dy;
    m_total_cells_num = abs_dx + 1;  // There are more x-values than y-values.
  } else {
    // There is at least one y-value for every x-value
    m_increment2.x() = 0;  // Do not change the x for every iteration.
    m_increment1.y() = 0;  // Do not change the y when numerator >= denominator.
    m_denominator = abs_dy;
    m_numerator = abs_dy / 2;
    m_numerator_add = abs_dx;
    m_total_cells_num = abs_dy + 1;  // There are more y-values than x-values.
  }
}

bool LineIterator::operator!=(const LineIterator & other) const
{
  return m_current_coord.x() != other.m_current_coord.x() && m_current_coord.y() != other.m_current_coord.y();
}

const wombat_core::grid_coord_t & LineIterator::operator*() const
{
  return m_current_coord;
}

LineIterator & LineIterator::operator++()
{
  m_numerator += m_numerator_add;  // Increase the numerator by the top of the fraction.
  if (m_numerator >= m_denominator) {
    m_numerator -= m_denominator;
    m_current_coord.x() += m_increment1.x();
    m_current_coord.y() += m_increment1.y();
  }
  m_current_coord.x() += m_increment2.x();
  m_current_coord.y() += m_increment2.y();
  ++m_current_cell_count;
  return *this;
}

bool LineIterator::is_past_end() const
{
  return m_current_cell_count >= m_total_cells_num;
}

}  // namespace wombat_core
