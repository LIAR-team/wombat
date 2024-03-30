// Copyright 2024 Soragna Alberto.

#pragma once

#include <limits>

#include "Eigen/Core"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

namespace wombat_core
{

/** @brief The index to access elements on a grid when represented as a linear vector */
using grid_index_t = size_t;

/** @brief The coordinates of a cell on a 2D grid */
using grid_coord_t = Eigen::Array2i;

/** @brief size of each dimension of a 2D grid */
using grid_size_t = grid_coord_t;

/**
 * @brief Class storing map metadata.
 * It can be used as a bridge between multiple map representations.
 */
class MapMetaDataAdapter
{
public:
  MapMetaDataAdapter() = default;
  ~MapMetaDataAdapter() = default;
  MapMetaDataAdapter(const MapMetaDataAdapter & other) = default;
  MapMetaDataAdapter & operator=(const MapMetaDataAdapter & other) = default;
  MapMetaDataAdapter & operator=(MapMetaDataAdapter && other) = default;

  explicit MapMetaDataAdapter(const nav_msgs::msg::MapMetaData & map_info)
  {
    static constexpr auto MAX_SCALAR = static_cast<uint32_t>(std::numeric_limits<grid_size_t::Scalar>::max());
    if (map_info.width >= MAX_SCALAR || map_info.height >= MAX_SCALAR) {
      throw std::runtime_error("Input grid is too big for grid adapter");
    }
    grid_size = {
      static_cast<grid_size_t::Scalar>(map_info.width),
      static_cast<grid_size_t::Scalar>(map_info.height),
    };
    resolution = map_info.resolution;
    origin = map_info.origin;
  }

  wombat_core::grid_index_t num_grid_cells()
  {
    return
      static_cast<wombat_core::grid_index_t>(grid_size.x()) * static_cast<wombat_core::grid_index_t>(grid_size.y());
  }

  grid_size_t grid_size {0, 0};
  float resolution {0.0};
  geometry_msgs::msg::Pose origin;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace wombat_core
