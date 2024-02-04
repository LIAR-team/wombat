// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>

#include "wombat_core/math/online_statistics.hpp"
#include "wombat_core/math/statistics.hpp"

class TestOnlineStatisticsWithparams
  : public testing::TestWithParam<std::vector<double>>
{};

INSTANTIATE_TEST_SUITE_P(
  TestOnlineStatistics,
  TestOnlineStatisticsWithparams,
  ::testing::Values(
    std::vector<double>({2.5, 2.5, 2.5, 2.5}),
    std::vector<double>({1.0, -1.0, 1.0, -1.0, 1.0, -1.0}),
    std::vector<double>({0.0001, -0.000031, 1.1123141, -3.112121, 0.0000794}),
    std::vector<double>({867131.4, -0.000031, 76931.912, 1231.11417, -41.11314})
));

TEST_P(TestOnlineStatisticsWithparams, OnlineOfflineComparison)
{
  auto total_population = GetParam();

  // This test case doesn't support empty population
  ASSERT_FALSE(total_population.empty());

  wombat_core::OnlineStatistics stats;
  for (size_t i = 0; i < total_population.size(); i++) {
    double this_sample = total_population[i];
    stats.add_sample(this_sample);

    const std::vector<double> current_population(total_population.begin(), total_population.begin() + i + 1);
    static constexpr double EPSILON = 0.0005;
    EXPECT_NEAR(stats.mean(), wombat_core::compute_mean(current_population), EPSILON);
    if (current_population.size() > 1) {
      EXPECT_NEAR(stats.variance(), wombat_core::compute_variance(current_population), EPSILON);
    }
  }

  EXPECT_EQ(total_population.size(), stats.n());
  EXPECT_EQ(*(std::max_element(total_population.begin(), total_population.end())), stats.max());
  EXPECT_EQ(*(std::min_element(total_population.begin(), total_population.end())), stats.min());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
