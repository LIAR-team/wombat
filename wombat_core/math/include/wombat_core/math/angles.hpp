// Copyright 2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <type_traits>

#include "geometry_msgs/msg/quaternion.hpp"

namespace wombat_core
{

constexpr double PI = 3.141592653589793;
constexpr float PI_F = static_cast<float>(PI);

/**
 * @brief Converts an angle from radians to degrees
 * @tparam T scalar type used to represent the input angle and the result (e.g. double)
 * @param rad_angle angle in radians
 * @return constexpr T angle in degrees
 */
template<typename T>
constexpr T rad_to_deg(T rad_angle) noexcept
{
  static_assert(std::is_floating_point<T>::value, "rad_to_deg requires floating point args");
  return rad_angle * static_cast<T>(180.0) / static_cast<T>(PI);
}

/**
 * @brief Converts an angle from degrees to radians
 * @tparam T scalar type used to represent the input angle and the result (e.g. double)
 * @param rad_angle angle in degrees
 * @return constexpr T angle in radians
 */
template<typename T>
constexpr T deg_to_rad(T deg_angle) noexcept
{
  static_assert(std::is_floating_point<T>::value, "deg_to_rad requires floating point args");
  return deg_angle * static_cast<T>(PI) / static_cast<T>(180.0);
}

/**
 * @brief Wrap an angle to the range [-pi, pi].
 * See [On wrapping the Kalman filter and estimating with the SO(2) group](https://arxiv.org/pdf/1708.05551.pdf)
 * @tparam T scalar type used to represent the input angle and the result (e.g. double)
 * @param angle angle to wrap in radians
 * @return constexpr T wrapped angle in [-pi, pi]
 */
template<typename T>
constexpr T wrap_angle(T angle) noexcept
{
  static_assert(std::is_floating_point<T>::value, "wrap_angle requires floating point args");

  while (angle > static_cast<T>(PI)) {
    angle -= 2.0 * static_cast<T>(PI);
  }
  while (angle < -static_cast<T>(PI)) {
    angle += 2.0 * static_cast<T>(PI);
  }
  return angle;
}

/**
 * @brief Compute the difference between two angles in the range [-pi, pi].
 * Input parameters don't need to be wrapped.
 * See [On wrapping the Kalman filter and estimating with the SO(2) group](https://arxiv.org/pdf/1708.05551.pdf)
 * @tparam T scalar type used to represent the input angles and the result (e.g. double)
 * @param angle1 angle to subtract from
 * @param angle2 angle to subtract
 * @return constexpr T difference angle wrapped in [-pi, pi]
 */
template<typename T>
constexpr T angles_difference(T angle1, T angle2) noexcept
{
  static_assert(std::is_floating_point<T>::value, "angles_difference requires floating point args");
  return wrap_angle(angle1 - angle2);
}

/**
 * @brief Compute the shortest angle for going from one input to the other.
 * Input parameters don't need to be wrapped.
 * @tparam T scalar type used to represent the input angles and the result (e.g. double)
 * @param angle1 angle to start from
 * @param angle2 angle to reach
 * @return constexpr T shortest angular distance wrapped in [-pi, pi]
 */
template<typename T>
constexpr T shortest_angular_distance(T angle_from, T angle_to) noexcept
{
  static_assert(std::is_floating_point<T>::value, "shortest_angular_distance requires floating point args");
  return angles_difference(angle_to, angle_from);
}

/**
 * @brief Builds a quaternion message from roll, pitch and yaw angles.
 * @param roll the roll angle
 * @param pitch the pitch angle
 * @param yaw the yaw angle
 * @return geometry_msgs::msg::Quaternion
 */
geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw);

}  // namespace wombat_core
