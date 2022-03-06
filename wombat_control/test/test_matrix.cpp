// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include <gtest/gtest.h>

#include <iostream>

#include "wombat_control/data_types/matrix.hpp"

TEST(test_matrix, initialization_and_fill)
{
  Matrix matrix = Matrix(2, 2);

  matrix.fill(5);
  matrix[0][0] = 10;
  matrix[1][1] = 2.5;

  EXPECT_EQ(matrix[0][0], 10);
  EXPECT_EQ(matrix[0][1], 5);
  EXPECT_EQ(matrix[1][0], 5);
  EXPECT_EQ(matrix[1][1], 2.5);

  EXPECT_EQ(matrix.rows(), 2u);
  EXPECT_EQ(matrix.columns(), 2u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
