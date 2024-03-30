// Copyright 2024 Soragna Alberto.

#include <algorithm>
#include <vector>

#include "wombat_core/math/statistics.hpp"

namespace wombat_core
{

void remove_outliers(std::vector<double> & data)
{
  // Sort the data
  std::sort(data.begin(), data.end());

  // Calculate the first and third quartiles
  const size_t n = data.size();
  const size_t q1_index = n / 4;
  const size_t q3_index = (3 * n) / 4;

  const double q1 = data[q1_index];
  const double q3 = data[q3_index];

  // Calculate the interquartile range (IQR)
  const double iqr = q3 - q1;

  // Set the lower and upper bounds for outliers
  const double lower_bound = q1 - 1.5 * iqr;
  const double upper_bound = q3 + 1.5 * iqr;

  // Remove outliers
  for (auto iter = data.begin(); iter != data.end(); ) {
    const double value = *iter;
    if (value < lower_bound || value > upper_bound) {
      iter = data.erase(iter);
    } else {
      ++iter;
    }
  }
}

}  // namespace wombat_core
