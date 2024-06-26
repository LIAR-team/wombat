// Copyright 2024 Soragna Alberto.

/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "Eigen/Core"

#include "nav_msgs/msg/map_meta_data.hpp"

#include "wombat_core/grid/coordinates.hpp"

namespace wombat_core
{

/*!
 * Iterator class to iterate over a line in the map.
 * Based on Bresenham Line Drawing algorithm.
 */
class LineIterator
{
public:
  /*!
   * Constructor.
   * @param map_info information about the grid to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   */
  LineIterator(
    const MapMetaDataAdapter & map_info,
    const wombat_core::grid_coord_t & start,
    const wombat_core::grid_coord_t & end);

  /**
   * @brief Compare to another iterator.
   * @param other the other iterator
   * @return true if this iterator points to a different grid coordinate than the other one.
   */
  bool operator!=(const LineIterator & other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const wombat_core::grid_coord_t & operator*() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  LineIterator & operator++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool is_past_end() const;

private:
  //! Current index.
  wombat_core::grid_coord_t m_current_coord;

  //! Starting index of the line.
  wombat_core::grid_coord_t m_start_coord;

  //! Ending index of the line.
  wombat_core::grid_coord_t m_end_coord;

  //! Current cell number.
  size_t m_current_cell_count {0};

  //! Number of cells in the line.
  size_t m_total_cells_num {0};

  //! Helper variables for Bresenham Line Drawing algorithm.
  grid_size_t m_increment1;
  grid_size_t m_increment2;
  int m_denominator{0};
  int m_numerator{0};
  int m_numerator_add{0};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace wombat_core
