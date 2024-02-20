// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "wombat_core/grid/coordinates.hpp"

#include <iostream>

namespace wombat_core
{

bool grid_coord_is_valid(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  return
    grid_coord.x() >= 0 && grid_coord.x() < map_info.grid_size.x() &&
    grid_coord.y() >= 0 && grid_coord.y() < map_info.grid_size.y();
}

bool grid_index_is_valid(
  const grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info)
{
  return
    grid_index < static_cast<grid_index_t>(map_info.grid_size.x()) * static_cast<grid_index_t>(map_info.grid_size.y());
}

std::optional<grid_index_t> grid_coord_to_index(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  if (!grid_coord_is_valid(grid_coord, map_info)) {
    return std::nullopt;
  }
  return grid_coord.y() * map_info.grid_size.x() + grid_coord.x();
}

std::optional<grid_coord_t> grid_index_to_coord(
  const grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info)
{
  std::cout << "------ START grid_index_to_coord --> "<<grid_index<<std::endl;


  if (!grid_index_is_valid(grid_index, map_info)) {
    return std::nullopt;
  }
  //grid_coord_t grid_coord;
  //grid_coord.y = grid_index / map_info.width;
  //grid_coord.x = grid_index - static_cast<grid_index_t>(grid_coord.y * map_info.width);

  grid_coord_t::Scalar y_coord = static_cast<grid_coord_t::Scalar>(static_cast<grid_coord_t::Scalar>(grid_index) / map_info.grid_size.x());
    /*
  grid_coord_t grid_coord {
    static_cast<grid_coord_t::Scalar>(static_cast<grid_coord_t::Scalar>(grid_index) - y_coord * map_info.grid_size.x()),
    static_cast<grid_coord_t::Scalar>(static_cast<grid_coord_t::Scalar>(grid_index) / map_info.grid_size.x()),
  };

  std::cout << "------ grid_index_to_coord --> idx "<<grid_index<< " computed coord "<<
  (static_cast<grid_coord_t::Scalar>(grid_index) - y_coord * map_info.grid_size.x()) << " " <<
  (static_cast<grid_coord_t::Scalar>(grid_index) / map_info.grid_size.x())
  << " vals " << grid_coord.x() << " " << y_coord<< std::endl;
  std::cout << "------ grid_index_to_coord --> finish idx "<<grid_index<< " First term "<<
  (y_coord * map_info.grid_size.x())<<std::endl;
  */
  return grid_coord_t{
    static_cast<grid_coord_t::Scalar>(static_cast<grid_coord_t::Scalar>(grid_index) - y_coord * map_info.grid_size.x()),
    y_coord
  };
}

std::optional<grid_coord_t> world_pt_to_grid_coord(
  const geometry_msgs::msg::Point & world_pt,
  const MapMetaDataAdapter & map_info)
{
  const auto & map_origin_x = map_info.origin.position.x;
  const auto & map_origin_y = map_info.origin.position.y;

  if (world_pt.x < map_origin_x || world_pt.y < map_origin_y) {
    return std::nullopt;
  }
  const float map_resolution = map_info.resolution;
  grid_coord_t grid_coord {
    static_cast<grid_coord_t::Scalar>((world_pt.x - map_origin_x) / map_resolution),
    static_cast<grid_coord_t::Scalar>((world_pt.y - map_origin_y) / map_resolution)
  };

  if (grid_coord.x() >= map_info.grid_size.x() || grid_coord.y() >= map_info.grid_size.y()) {
    return std::nullopt;
  }

  return grid_coord;
}

std::optional<geometry_msgs::msg::Point> grid_coord_to_world_pt(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  std::cout << "------ START grid_coord_to_world_pt --> "<<grid_coord.x() << " " << grid_coord.y()<<std::endl;

  if (!grid_coord_is_valid(grid_coord, map_info)) {
    std::cout<<"grid_coord_to_world_pt Value is invalid" << std::endl;
    return std::nullopt;
  }

  geometry_msgs::msg::Point world_pt;
  world_pt.x = map_info.origin.position.x + (grid_coord.x() + 0.5) * map_info.resolution;
  world_pt.y = map_info.origin.position.y + (grid_coord.y() + 0.5) * map_info.resolution;
  std::cout << "------ grid_coord_to_world_pt got --> "<<world_pt.x << " " << world_pt.y<<std::endl;

  return world_pt;
}

std::optional<grid_index_t> world_pt_to_grid_index(
  const geometry_msgs::msg::Point & world_pt,
  const MapMetaDataAdapter & map_info)
{
  const auto maybe_grid_coord = world_pt_to_grid_coord(world_pt, map_info);
  if (!maybe_grid_coord) {
    return std::nullopt;
  }
  return grid_coord_to_index(*maybe_grid_coord, map_info);
}

std::optional<geometry_msgs::msg::Point> grid_index_to_world_pt(
  const grid_index_t & grid_index,
  const MapMetaDataAdapter & map_info)
{
  std::cout << "--- START grid_index_to_world_pt --> "<<grid_index<<std::endl;
  const auto maybe_grid_coord = grid_index_to_coord(grid_index, map_info);
  std::cout << "grid_index_to_world_pt got coord? --> "<< static_cast<int>(maybe_grid_coord != std::nullopt)<<std::endl;
  if (!maybe_grid_coord) {
    return std::nullopt;
  }
  std::cout << "grid_index_to_world_pt got coord --> "<< maybe_grid_coord->x() << " " << maybe_grid_coord->y() << std::endl;
  return grid_coord_to_world_pt(*maybe_grid_coord, map_info);
}

grid_coord_t grid_coord_bounded_diff(
  const grid_coord_t & minuend,
  const grid_coord_t & subtrahend)
{
  auto difference = minuend - subtrahend;
  return grid_coord_t{
    std::max(difference.x(), 0),
    std::max(difference.y(), 0)
  };
}

grid_coord_t grid_coord_bounded_sum(
  const grid_coord_t & addend_1,
  const grid_coord_t & addend_2,
  const MapMetaDataAdapter & map_info)
{
  auto sum = addend_1 + addend_2;
  return grid_coord_t{
    std::min(sum.x(), map_info.grid_size.x() - 1),
    std::min(sum.y(), map_info.grid_size.y() - 1)
  };
}

static grid_coord_t::Scalar enfouce_bounds_on_grid_coord_dimension(
  grid_coord_t::Scalar coord_scalar,
  grid_size_t::Scalar size)
{
  if (coord_scalar >= size) {
    return size - 1;
  }
  return coord_scalar;
}

grid_coord_t enfouce_bounds_on_grid_coord(
  const grid_coord_t & grid_coord,
  const MapMetaDataAdapter & map_info)
{
  return grid_coord_t{
    enfouce_bounds_on_grid_coord_dimension(grid_coord.x(), map_info.grid_size.x()),
    enfouce_bounds_on_grid_coord_dimension(grid_coord.y(), map_info.grid_size.y()),
  };
}

grid_size_t get_subgrid_size_from_corners(
  const grid_coord_t & top_left_grid_coord,
  const grid_coord_t & bottom_right_grid_coord)
{
  return grid_size_t{
    bottom_right_grid_coord.x() - top_left_grid_coord.x() + 1,
    bottom_right_grid_coord.y() - top_left_grid_coord.y() + 1
  };
}

bool increment_index_for_submap(
  grid_coord_t & submapIndex,
  grid_coord_t & index,
  const grid_coord_t & submapTopLeftIndex,
  const MapMetaDataAdapter & submap_info)
{
  // Copy the data first, only copy it back if everything is within range.
  grid_coord_t tempSubmapIndex = submapIndex;

  // Increment submap index.
  if (tempSubmapIndex.x() < submap_info.grid_size.x() - 1) {
    // Same row.
    tempSubmapIndex.x()++;
  } else {
    // Next row.
    tempSubmapIndex.y()++;
    tempSubmapIndex.x() = 0;
  }

  // End of iterations reached.
  if (!grid_coord_is_valid(tempSubmapIndex, submap_info)) {
    //std::cout<<"Reached invalid coord from "<< submapIndex.x << " "<< submapIndex.y<< " to " << tempSubmapIndex.x << " " << tempSubmapIndex.y << " inside " << submap_info.width << " " << submap_info.height << std::endl;
    return false;
  }

  // Copy data back.
  index = submapTopLeftIndex + tempSubmapIndex;
  submapIndex = tempSubmapIndex;
  return true;
}

}  // namespace wombat_core
