// Copyright 2024 Soragna Alberto.

#pragma once

#include <cmath>
#include <cstdint>

namespace wombat_core
{

/**
 * @brief Online statistics aggregator (mean, variance, etc).
 * The variance computation is based on the Welford's Online Algorithm.
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
 */
class OnlineStatistics
{
public:
  OnlineStatistics() = default;

  /**
   * @brief Add a new sample to the statistic aggregator.
   * @param new_sample the new sample
   */
  void add_sample(double new_sample);

  /**
   * @brief Get the current mean estimate.
   * @return The mean
   */
  double mean() const;

  /**
   * @brief Get the current variance estimate.
   * @note This is the "population variance" (1/n), not the "sample variance" (1/n-1)
   * @return The variance (or nan if unable to compute it)
   */
  double variance() const;

  /**
   * @brief Get the current standard deviation estimate
   * @note This is the "population std dev" (1/n), not the "sample std dev" (1/n-1)
   * @return The standard deviation (or nan if unable to compute it)
   */
  double stddev() const;

  /**
   * @brief Get the max sample added
   * @return The max sample (or nan if no samples were added)
   */
  double max() const;

  /**
   * @brief Get the min sample added
   * @return The min sample (or nan if no samples were added)
   */
  double min() const;

  /**
   * @brief Get the number of processed samples
   * @return The number of samples
   */
  uint64_t n() const;

private:
  double m_max {0.0};
  double m_min {0.0};
  double m_mean {0.0};
  double m_m2 {0.0};
  uint64_t m_num_samples {0};
};

}  // namespace wombat_core
