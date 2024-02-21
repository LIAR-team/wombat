/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <memory>
#include <vector>

#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/submap_iterator.hpp"

namespace wombat_core
{

/*!
 * Iterator class to iterate through a polygonal area of the map.
 */
class PolygonIterator
{
public:
  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param polygon the polygonal area to iterate on.
   */
  PolygonIterator(const MapMetaDataAdapter & map_info, const std::vector<grid_coord_t> & polygon);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  PolygonIterator & operator=(const PolygonIterator & other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator!=(const PolygonIterator & other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const grid_coord_t & operator*() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  PolygonIterator & operator++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool is_past_end() const;

private:
  /*!
   * Check if current index is inside polygon.
   * @return true if inside, false otherwise.
   */
  bool isInside() const;

  //! Polygon to iterate on.
  std::vector<grid_coord_t> polygon_;

  //! Grid submap iterator.
  std::unique_ptr<SubmapIterator> internalIterator_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace wombat_core
