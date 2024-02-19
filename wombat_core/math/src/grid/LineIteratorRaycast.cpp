/*
 * LineIteratorRaycast.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "wombat_core/math/grid/LineIteratorRaycast.hpp"
#include "wombat_core/cpp/types.hpp"
#include "wombat_core/math/utils.hpp"
#include "wombat_core/math/grid/coordinates.hpp"

namespace wombat_core {

LineIteratorRaycast::LineIteratorRaycast(const nav_msgs::msg::MapMetaData & map_info, const Index& start, const Index& end)
{
  initialize(map_info, start, end);
}

bool LineIteratorRaycast::operator !=(const LineIteratorRaycast& other) const
{
  return m_current_coord.x != other.m_current_coord.x && m_current_coord.y != other.m_current_coord.y;
}

unsigned int LineIteratorRaycast::operator *() const
{
  return m_offset;
}

LineIteratorRaycast& LineIteratorRaycast::operator ++()
{
  m_offset += m_offset_a;
  m_error_b += static_cast<int>(m_abs_db);
  if (static_cast<unsigned int>(m_error_b) >= m_abs_da) {
    m_offset += m_offset_b;
    m_error_b -= static_cast<int>(m_abs_da);
  }

  ++iCell_;
  return *this;
}

bool LineIteratorRaycast::isPastEnd() const
{
  return iCell_ >= m_end;
}

bool LineIteratorRaycast::initialize(const nav_msgs::msg::MapMetaData & map_info, const Index& start, const Index& end)
{
    start_ = start;
    end_ = end;
    bufferSize_ = {map_info.width, map_info.height};
    bufferStartIndex_ = {0, 0};
    initializeIterationParameters(map_info);
    return true;
}

void LineIteratorRaycast::initializeIterationParameters(const nav_msgs::msg::MapMetaData & map_info)
{
  iCell_ = 0;

  double_range_t length_range = {0.0, std::numeric_limits<double>::max()};

  const int dx_full = static_cast<int>(end_.x() - start_.x());
  const int dy_full = static_cast<int>(end_.y() - start_.y());

  // we need to chose how much to scale our dominant dimension,
  // based on the maximum length of the line
  const double dist = std::hypot(dx_full, dy_full);
  if (dist < length_range.min) {
    assert(0 && "dist less than");
    //return std::nullopt;
  }

  grid_coord_t min_from_grid;
  if (dist > 0.0) {
    // Adjust starting point and offset to start from length_range.min distance
    min_from_grid.x = static_cast<unsigned int>(start_.x() + dx_full / dist * length_range.min);
    min_from_grid.y = static_cast<unsigned int>(start_.y() + dy_full / dist * length_range.min);
  } else {
    // dist can be 0 if [start_.x(), start_.y()]==[to_grid.x, to_grid.y].
    // In this case only this cell should be processed.
    min_from_grid.x = start_.x();
    min_from_grid.y = start_.y();
  }
  const auto maybe_from_offset = wombat_core::grid_coord_to_index(min_from_grid, map_info);
  if (!maybe_from_offset) {
    assert(0 && "failed from offset");
    //throw std::runtime_error("Failed to compute from offset");
  }

  const int dx = dx_full;
  const int dy = dy_full;

  const unsigned int abs_dx = std::abs(dx);
  const unsigned int abs_dy = std::abs(dy);

  const int offset_dx = wombat_core::sign(dx);
  const int offset_dy = wombat_core::sign(dy) * static_cast<int>(map_info.width);

  const double scale = (dist == 0.0) ? 1.0 : std::min(1.0, length_range.max / dist);

  unsigned int max_length {0u};
  if (abs_dx >= abs_dy) {
    // if x is dominant
    const int error_y = static_cast<int>(abs_dx) / 2;
    m_abs_da = abs_dx;
    m_abs_db = abs_dy;
    m_error_b = error_y;
    m_offset_a = offset_dx;
    m_offset_b = offset_dy;
    m_offset = *maybe_from_offset;
    max_length = static_cast<unsigned int>(scale * abs_dx);
  } else {
    // otherwise y is dominant
    const int error_x = static_cast<int>(abs_dy) / 2;
    m_abs_da = abs_dy;
    m_abs_db = abs_dx;
    m_error_b = error_x;
    m_offset_a = offset_dy;
    m_offset_b = offset_dx;
    m_offset = *maybe_from_offset;
    max_length = static_cast<unsigned int>(scale * abs_dy);
  }

  m_end = std::min(max_length, m_abs_da);
}

} /* namespace grid_map */
