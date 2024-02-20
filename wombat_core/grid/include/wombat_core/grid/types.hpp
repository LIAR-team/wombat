// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <limits>

#include "Eigen/Core"

namespace wombat_core
{

/** @brief The index to access elements on a grid */
using grid_index_t = size_t;

/** @brief The coordinates of a cell on a grid */
using grid_coord_t = Eigen::Array2i;

using grid_size_t = grid_coord_t;

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

  grid_size_t grid_size {0, 0};
  float resolution {0.0};
  geometry_msgs::msg::Pose origin;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace wombat_core
