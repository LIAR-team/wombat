/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 12, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "nav_msgs/msg/map_meta_data.hpp"

#include "wombat_core/grid/coordinates.hpp"

#include <Eigen/Core>

namespace wombat_core {

/*!
 * Iterator class to iterate through a rectangular part of the map (submap).
 * Before using this iterator, make sure that the requested submap is
 * actually contained in the grid map.
 */
class SubmapIterator
{
public:
  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param submapStartIndex the start index of the submap, typically top-left index.
   * @param submapSize the size of the submap to iterate on.
   */
  SubmapIterator(
    MapMetaDataAdapter map_info,
    const wombat_core::grid_coord_t & start,
    const wombat_core::grid_size_t & submap_size);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const SubmapIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const grid_coord_t & operator *() const;

  // Overloading the arrow operator (->)
  grid_coord_t* operator->()
  {
    return &m_current_coord;
  }

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  SubmapIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;
private:

  MapMetaDataAdapter m_map_info;
  MapMetaDataAdapter m_submap_info;

  //! Current coord.
  grid_coord_t m_current_coord;

  //! Submap buffer size.
  grid_size_t submapSize_;

  //! Top left coord of the submap.
  grid_coord_t m_submap_top_left_coord;

  //! Current index in the submap.
  grid_coord_t m_current_submap_coord;

  //! Is iterator out of scope.
  bool isPastEnd_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace grid_map
