// Copyright 2024 Soragna Alberto.

/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 12, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "Eigen/Core"

#include "nav_msgs/msg/map_meta_data.hpp"

#include "wombat_core/grid/coordinates.hpp"

namespace wombat_core
{

/**
 * @brief Iterator class to iterate through a rectangular part of the map (submap).
 * Before using this iterator, make sure that the requested submap is
 * actually contained in the grid map.
 */
class SubmapIterator
{
public:
  /**
   * @brief Constructor.
   * @param map_info information about the grid on which to iterate
   * @param start the start grid coord of the submap, i.e. its minimum corner
   * @param submap_size the size of the submap to iterate on.
   */
  SubmapIterator(
    const MapMetaDataAdapter & map_info,
    const grid_coord_t & start,
    const grid_size_t & submap_size);

  /**
   * @brief Compare to another iterator.
   * @param other the other iterator
   * @return true if this iterator points to a different grid coordinate than the other one.
   */
  bool operator!=(const SubmapIterator & other) const;

  /**
   * @brief Dereference the iterator with const.
   * @return the grid coordinate to which the iterator is pointing.
   */
  const grid_coord_t & operator*() const;

  // Overloading the arrow operator (->)

  /**
   * @brief Overloaded arrow operator
   * @return the grid coordinate to which the iterator is pointing.
   */
  grid_coord_t * operator->()
  {
    return &m_current_coord;
  }

  /**
   * @brief Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  SubmapIterator & operator++();

  /**
   * @brief Indicates if iterator is past end.
   * @return true if iterator is out of scope
   */
  bool is_past_end() const;

private:
  MapMetaDataAdapter m_map_info;
  MapMetaDataAdapter m_submap_info;

  //! Current coord.
  grid_coord_t m_current_coord;

  //! Minimum coord of the submap.
  grid_coord_t m_submap_min_coord;

  //! Current index in the submap.
  grid_coord_t m_current_submap_coord;

  //! Is iterator out of scope.
  bool m_is_past_end {false};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace wombat_core
