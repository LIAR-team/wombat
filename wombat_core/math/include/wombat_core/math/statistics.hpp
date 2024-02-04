// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace wombat_core
{

/**
 * @brief Compute the mean (average value) among the elements in a container
 * @tparam ContainerT The container type (must follow STL syntax)
 * @param population the samples over which to compute the mean
 * @return The mean (casted to the container value type)
 */
template<typename ContainerT>
inline typename ContainerT::value_type compute_mean(const ContainerT & population)
{
  using T = typename ContainerT::value_type;
  if (population.empty()) {
    return static_cast<T>(0);
  }
  const double count = static_cast<T>(population.size());
  return std::reduce(population.begin(), population.end()) / count;
}

/**
 * @brief Compute the population variance among the elements in a container
 * @tparam ContainerT The container type (must follow STL syntax)
 * @param population the samples over which to compute the variance
 * @return The variance (casted to the container value type)
 */
template<typename ContainerT>
inline typename ContainerT::value_type compute_variance(const ContainerT & population)
{
  using T = typename ContainerT::value_type;
  if (population.empty()) {
    return std::nan("");
  }

  const T mean = compute_mean(population);
  std::vector<T> deltas(population.size());
  std::transform(
    population.begin(), population.end(), deltas.begin(),
    [mean](T x) {return x - mean;});

  const T sq_sum = std::inner_product(deltas.begin(), deltas.end(), deltas.begin(), static_cast<T>(0));
  const T variance = sq_sum / static_cast<T>(population.size());
  return variance;
}

/**
 * @brief Removes outliers from a population using the IQR method.
 * @param data the population: this will be modified in-place
 */
void remove_outliers(std::vector<double> & data);

}  // namespace wombat_core
