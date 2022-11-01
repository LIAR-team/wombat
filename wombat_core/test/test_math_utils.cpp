// Copyright 2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include "wombat_core/math/utils.hpp"

TEST(test_utils, clamp_to_zero)
{
  using wombat_core::clamp_to_zero;

  EXPECT_EQ(1.0, clamp_to_zero(1.0, 0.1));
  EXPECT_EQ(-1.0, clamp_to_zero(-1.0, 0.1));
  EXPECT_EQ(1e-2, clamp_to_zero(1e-2, 1e-5));

  EXPECT_EQ(0.0, clamp_to_zero(1.0, 1.1));
  EXPECT_EQ(0.0, clamp_to_zero(-1.0, 1.1));
  EXPECT_EQ(0.0, clamp_to_zero(1e-5, 1e-2));

  EXPECT_EQ(0, clamp_to_zero(4, 10));
  EXPECT_EQ(-12, clamp_to_zero(-12, 10));
  EXPECT_EQ(0, clamp_to_zero(-50, 60));
}

TEST(test_utils, sign)
{
  using wombat_core::sign;

  EXPECT_EQ(1, sign(1));
  EXPECT_EQ(1, sign(12.12));

  EXPECT_EQ(-1, sign(-1));
  EXPECT_EQ(-1, sign(-12.12));

  EXPECT_EQ(1, sign(0));
  EXPECT_EQ(1, sign(0.0f));
  EXPECT_EQ(1, sign(-0));
  EXPECT_EQ(1, sign(-0.0f));
}

TEST(test_utils, sign_with_zero)
{
  using wombat_core::sign_with_zero;

  EXPECT_EQ(1, sign_with_zero(1));
  EXPECT_EQ(1, sign_with_zero(12.12));

  EXPECT_EQ(-1, sign_with_zero(-1));
  EXPECT_EQ(-1, sign_with_zero(-12.12));

  EXPECT_EQ(0, sign_with_zero(0));
  EXPECT_EQ(0, sign_with_zero(0.0f));
  EXPECT_EQ(0, sign_with_zero(-0));
  EXPECT_EQ(0, sign_with_zero(-0.0f));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
