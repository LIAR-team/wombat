// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace wombat_core
{

// Hack to get around gcc bug
// default member initializer required before the end of its enclosing class
namespace detail
{
struct config_t
{
  double inflation_radius {0.55};
  double cost_scaling_factor {10.0};
};
}  // namespace detail

class InflationLayer : public nav2_costmap_2d::InflationLayer
{
public:
  using config_t = detail::config_t;

  InflationLayer() = default;

  void setup(
    nav2_costmap_2d::LayeredCostmap * parent,
    const config_t & config = config_t());
};

}  // namespace wombat_core
