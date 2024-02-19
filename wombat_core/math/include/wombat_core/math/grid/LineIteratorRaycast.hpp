/*
 * LineIteratorRaycast.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <Eigen/Core>
#include "nav_msgs/msg/map_meta_data.hpp"

#include "wombat_core/math/grid/coordinates.hpp"

namespace wombat_core {

using Index = Eigen::Array2i;
using Size = Eigen::Array2i;

/*!
 * Iterator class to iterate over a line in the map.
 * Based on Bresenham Line Drawing algorithm.
 */
class LineIteratorRaycast
{
public:
  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   */
  LineIteratorRaycast(const nav_msgs::msg::MapMetaData & map_info, const Index& start, const Index& end);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const LineIteratorRaycast& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  unsigned int operator *() const;
  //const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  LineIteratorRaycast& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:


  /*!
   * Construct function.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   * @return true if successful, false otherwise.
   */
  bool initialize(const nav_msgs::msg::MapMetaData & map_info, const Index& start, const Index& end);

  /*!
   * Computes the parameters requires for the line drawing algorithm.
   */
  void initializeIterationParameters(const nav_msgs::msg::MapMetaData & map_info);

  //! Current index.
  wombat_core::grid_coord_t m_current_coord;

  unsigned int m_abs_da;
  unsigned int m_abs_db;
  int m_error_b;
  int m_offset_a;
  int m_offset_b;
  unsigned int m_offset;
  unsigned int m_max_length;
  unsigned int m_end;

  //! Starting index of the line.
  Index start_;

  //! Ending index of the line.
  Index end_;

  //! Current cell number.
  unsigned int iCell_ = 0;

  //! Number of cells in the line.
  unsigned int nCells_ = 0;

  //! Helper variables for Bresenham Line Drawing algorithm.
  Size increment1_, increment2_;
  int denominator_{0}, numerator_{0}, numeratorAdd_{0};

  //! Map information needed to get position from iterator.
  Size bufferSize_;
  Index bufferStartIndex_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace grid_map
