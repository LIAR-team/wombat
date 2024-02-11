// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <string>

#include "wombat_core/costmap/inflation_layer.hpp"

namespace wombat_core
{

void InflationLayer::setup(
  nav2_costmap_2d::LayeredCostmap * parent,
  const config_t & config)
{
  name_ = "kennel_inflation";
  layered_costmap_ = parent;

  inflation_radius_ = config.inflation_radius;
  cost_scaling_factor_ = config.cost_scaling_factor;

  enabled_ = true;
  inflate_unknown_ = false;
  inflate_around_unknown_ = false;

  current_ = true;
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  matchSize();
}

}  // namespace wombat_core
