/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "wombat_core/math/grid/SubmapIterator.hpp"

namespace wombat_core {

static bool checkIfIndexInRange(const Index& index, const Size& bufferSize)
{
  return index[0] >= 0 && index[1] >= 0 && index[0] < bufferSize[0] && index[1] < bufferSize[1];
}

/*!
 * Increases the index by one to iterate through the cells of a submap.
 * Increments either to the neighboring index to the right or to
 * the start of the lower row. Returns false if end of iterations are reached.
 *
 * Note: This function does not check if submap actually fits to the map. This needs
 * to be checked before separately.
 *
 * @param[in/out] submapIndex the index in the submap that is incremented.
 * @param[out] index the index in the map that is incremented (corrected for the circular buffer).
 * @param[in] submapTopLefIndex the top left index of the submap.
 * @param[in] submapBufferSize the submap buffer size.
 * @param[in] bufferStartIndex the map buffer start index.
 * @return true if successfully incremented indices, false if end of iteration limits are reached.
 */
static bool incrementIndexForSubmap(
  Index& submapIndex, Index& index, const Index& submapTopLeftIndex,
  const Size& submapBufferSize)
{
  // Copy the data first, only copy it back if everything is within range.
  Index tempIndex = index;
  Index tempSubmapIndex = submapIndex;

  // Increment submap index.
  if (tempSubmapIndex[1] + 1 < submapBufferSize[1]) {
    // Same row.
    tempSubmapIndex[1]++;
  } else {
    // Next row.
    tempSubmapIndex[0]++;
    tempSubmapIndex[1] = 0;
  }

  // End of iterations reached.
  if (!checkIfIndexInRange(tempSubmapIndex, submapBufferSize)) {
    return false;
  }

  // Get corresponding index in map.
  tempIndex = submapTopLeftIndex + tempSubmapIndex;

  // Copy data back.
  index = tempIndex;
  submapIndex = tempSubmapIndex;
  return true;
}

SubmapIterator::SubmapIterator(
  const nav_msgs::msg::MapMetaData & map_info,
  const wombat_core::grid_coord_t & start,
  size_t submap_width,
  size_t submap_height)
{
  size_ = {map_info.width, map_info.height};
  startIndex_ = {0, 0};
  index_ = {static_cast<int>(start.x), static_cast<int>(start.y)};
  submapSize_ = {submap_width, submap_height};
  submapStartIndex_ = index_;
  submapIndex_.setZero();
  isPastEnd_ = false;
}

bool SubmapIterator::operator !=(const SubmapIterator& other) const
{
  return (index_ != other.index_).any();
}

const Index& SubmapIterator::operator *() const
{
  return index_;
}

SubmapIterator& SubmapIterator::operator ++()
{
  isPastEnd_ = !incrementIndexForSubmap(submapIndex_, index_, submapStartIndex_,
                                        submapSize_);
  return *this;
}

bool SubmapIterator::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace grid_map */
