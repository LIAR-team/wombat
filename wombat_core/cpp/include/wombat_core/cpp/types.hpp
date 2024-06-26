// Copyright 2024 Soragna Alberto.

#pragma once

#include <limits>

namespace wombat_core
{

struct double_range_t
{
  double min {std::numeric_limits<double>::lowest()};
  double max {std::numeric_limits<double>::max()};
};

}  // namespace wombat_core
