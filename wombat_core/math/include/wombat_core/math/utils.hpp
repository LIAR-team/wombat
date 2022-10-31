// Copyright 2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <cmath>

namespace wombat_core
{

/**
 * @brief Clamp (saturate) a value to zero if it's norm is smaller than the provided lower limit
 * NOTE: if the lower limit is negative, the function will always clamp
 * @tparam T type used to represent the value, the limit and the clamped value
 * @param value value to clamp
 * @param limit limit
 * @return constexpr T clamped value (either the input value or 0)
 */
template<typename T>
constexpr T clamp_to_zero(const T & value, const T & limit) noexcept
{
  if (std::abs(value) <= limit) {
    return 0;
  }
  return value;
}

/**
 * @brief Computes the sign of a value as +1 or -1
 * @tparam T type of the input value
 * @param value value to compute the sign of
 * @return constexpr int -1 if value is less than 0, +1 otherwise
 */
template<typename T>
constexpr int sign(const T & value) noexcept
{
  return value < static_cast<T>(0) ? -1 : 1;
}

/**
 * @brief Computes the sign of a value as +1, -1 or 0
 * @tparam T type of the input value
 * @param value value to compute the sign of
 * @return constexpr int -1 if value is negative, +1 if it's positive, 0 if it's 0
 */
template<typename T>
constexpr int sign_with_zero(const T & value) noexcept
{
  return static_cast<int>(static_cast<T>(0) < value) - static_cast<int>(value < static_cast<T>(0));
}

}  // namespace wombat_core
