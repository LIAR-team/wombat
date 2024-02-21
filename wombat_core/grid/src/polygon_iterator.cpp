/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <memory>

#include "wombat_core/grid/polygon_iterator.hpp"

namespace wombat_core
{

static std::pair<grid_coord_t, grid_coord_t> get_bounding_box(
  const std::vector<grid_coord_t> & points)
{
  std::pair<grid_coord_t, grid_coord_t> bbox;
  bbox.first = {
    std::numeric_limits<grid_coord_t::Scalar>::max(),
    std::numeric_limits<grid_coord_t::Scalar>::max()
  };
  bbox.second = {
    std::numeric_limits<grid_coord_t::Scalar>::lowest(),
    std::numeric_limits<grid_coord_t::Scalar>::lowest()
  };
  assert(!points.empty());
  for (const auto & pt : points) {
    bbox.first.x() = std::min(bbox.first.x(), pt.x());
    bbox.first.y() = std::min(bbox.first.y(), pt.y());
    bbox.second.x() = std::max(bbox.second.x(), pt.x());
    bbox.second.y() = std::max(bbox.second.y(), pt.y());
  }

  return bbox;
}

PolygonIterator::PolygonIterator(
  const MapMetaDataAdapter & map_info,
  const std::vector<grid_coord_t> & polygon)
: polygon_(polygon)
{
  auto bbox = get_bounding_box(polygon);
  assert(grid_coord_is_valid(bbox.first, map_info));
  assert(grid_coord_is_valid(bbox.second, map_info));

  auto bbox_size = get_grid_size_from_corners(bbox.first, bbox.second);

  internalIterator_ = std::make_unique<SubmapIterator>(map_info, bbox.first, bbox_size);
  if (!isInside()) {++(*this);}
}

PolygonIterator & PolygonIterator::operator=(const PolygonIterator & other)
{
  (void)other;
  /*
  polygon_ = other.polygon_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  */
  return *this;
}

bool PolygonIterator::operator!=(const PolygonIterator & other) const
{
  return internalIterator_ != other.internalIterator_;
}

const grid_coord_t & PolygonIterator::operator*() const
{
  return *(*internalIterator_);
}

PolygonIterator & PolygonIterator::operator++()
{
  ++(*internalIterator_);
  if (internalIterator_->is_past_end()) {
    return *this;
  }

  for (; !internalIterator_->is_past_end(); ++(*internalIterator_)) {
    if (isInside()) {
      break;
    }
  }

  return *this;
}

bool PolygonIterator::is_past_end() const
{
  return internalIterator_->is_past_end();
}

bool PolygonIterator::isInside() const
{
  return true;
  /*
  Position position;
  getPositionFromIndex(
    position, *(*internalIterator_), mapLength_, mapPosition_, resolution_,
    bufferSize_, bufferStartIndex_);
  return polygon_.isInside(position);
  */
}

}  // namespace grid_map
