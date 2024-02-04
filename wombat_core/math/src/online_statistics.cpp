// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <cmath>

#include "wombat_core/math/online_statistics.hpp"

namespace wombat_core
{

void OnlineStatistics::add_sample(double new_sample)
{
  if (m_num_samples == 0) {
    m_max = new_sample;
    m_min = new_sample;
  } else {
    if (new_sample > m_max) {
      m_max = new_sample;
    } else if (new_sample < m_min) {
      m_min = new_sample;
    }
  }

  m_num_samples++;
  const auto delta = new_sample - m_mean;
  m_mean += delta / static_cast<double>(m_num_samples);
  const auto delta2 = new_sample - m_mean;
  m_m2 += delta * delta2;
}

double OnlineStatistics::mean() const
{
  return m_mean;
}

double OnlineStatistics::variance() const
{
  if (m_num_samples == 0) {
    return std::nan("");
  }
  if (m_num_samples == 1) {
    return 0;
  }

  return m_m2 / static_cast<double>(m_num_samples);
}

double OnlineStatistics::stddev() const
{
  const auto variance = this->variance();
  if (std::isnan(variance)) {
    return variance;
  }
  return std::sqrt(variance);
}

double OnlineStatistics::max() const
{
  if (m_num_samples == 0) {
    return std::nan("");
  }
  return m_max;
}

double OnlineStatistics::min() const
{
  if (m_num_samples == 0) {
    return std::nan("");
  }
  return m_min;
}

uint64_t OnlineStatistics::n() const
{
  return m_num_samples;
}

}  // namespace wombat_core
