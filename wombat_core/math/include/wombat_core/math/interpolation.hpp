// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

namespace wombat_core
{

template<typename T>
T linear_interpolation(const T & start, const T & end, double factor)
{
  return start + static_cast<T>(factor * static_cast<double>(end - start));
}

}  // namespace wombat_core
