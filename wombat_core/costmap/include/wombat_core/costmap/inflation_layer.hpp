// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace wombat_core
{

class InflationLayer : public nav2_costmap_2d::InflationLayer
{
public:
  InflationLayer() = default;

  void setup(
    nav2_costmap_2d::LayeredCostmap * parent,
    double inflation_radius = 0.55,
    double cost_scaling_factor = 10.0);
};

}
