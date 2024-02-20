/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "wombat_core/grid/line_iterator.hpp"

namespace wombat_core {

LineIterator::LineIterator(
  const MapMetaDataAdapter & map_info,
  const wombat_core::grid_coord_t & start,
  const wombat_core::grid_coord_t & end)
{
  (void)map_info;

  start_ = start;
  end_ = end;

  iCell_ = 0;
  m_current_coord = start;

  if (end_.x() >= start_.x()) {
    // x-values increasing.
    increment1_.x() = 1;
    increment2_.x() = 1;
  } else {
    // x-values decreasing.
    increment1_.x() = -1;
    increment2_.x() = -1;
  }

  if (end_.y() >= start_.y()) {
    // y-values increasing.
    increment1_.y() = 1;
    increment2_.y() = 1;
  } else {
    // y-values decreasing.
    increment1_.y() = -1;
    increment2_.y() = -1;
  }

  const int dx = static_cast<int>(end_.x() - start_.x());
  const int dy = static_cast<int>(end_.y() - start_.y());
  const unsigned int abs_dx = std::abs(dx);
  const unsigned int abs_dy = std::abs(dy);

  if (abs_dx >= abs_dy) {
    // There is at least one x-value for every y-value.
    increment1_.x() = 0; // Do not change the x when numerator >= denominator.
    increment2_.y() = 0; // Do not change the y for every iteration.
    denominator_ = abs_dx;
    numerator_ = abs_dx / 2;
    numeratorAdd_ = abs_dy;
    nCells_ = abs_dx + 1; // There are more x-values than y-values.
  } else {
    // There is at least one y-value for every x-value
    increment2_.x() = 0; // Do not change the x for every iteration.
    increment1_.y() = 0; // Do not change the y when numerator >= denominator.
    denominator_ = abs_dy;
    numerator_ = abs_dy / 2;
    numeratorAdd_ = abs_dx;
    nCells_ = abs_dy + 1; // There are more y-values than x-values.
  }
}

bool LineIterator::operator !=(const LineIterator& other) const
{
  return m_current_coord.x() != other.m_current_coord.x() && m_current_coord.y() != other.m_current_coord.y();
}

const wombat_core::grid_coord_t & LineIterator::operator *() const
{
  return m_current_coord;
}

LineIterator& LineIterator::operator ++()
{
  numerator_ += numeratorAdd_;  // Increase the numerator by the top of the fraction.
  if (numerator_ >= denominator_) {
    numerator_ -= denominator_;
    m_current_coord.x() += increment1_.x();
    m_current_coord.y() += increment1_.y();
  }
  m_current_coord.x() += increment2_.x();
  m_current_coord.y() += increment2_.y();
  ++iCell_;
  return *this;
}

bool LineIterator::isPastEnd() const
{
  return iCell_ >= nCells_;
}

}  // namespace wombat_core
