// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#ifndef WOMBAT_CONTROL__DATA_TYPES__MATRIX_HPP_
#define WOMBAT_CONTROL__DATA_TYPES__MATRIX_HPP_

#include <iostream>
#include <vector>

struct Matrix
{
public:
  Matrix(size_t rows_n, size_t columns_n)
  {
    m_rows_n = rows_n;
    m_columns_n = columns_n;

    for (size_t i = 0; i < rows_n; i++) {
      std::vector<double> row_vec;

      for (size_t j = 0; j < columns_n; j++) {
        row_vec.push_back(0);
      }
      m_matrix.push_back(row_vec);
    }
  }

  const std::vector<double> & operator[](size_t row_index) const
  {
    return m_matrix[row_index];
  }

  std::vector<double> & operator[](size_t row_index)
  {
    return m_matrix[row_index];
  }

  size_t rows() const
  {
    return m_rows_n;
  }

  size_t columns() const
  {
    return m_columns_n;
  }

  void fill(double entry)
  {
    for (size_t i = 0; i < m_rows_n; i++) {
      for (size_t j = 0; j < m_columns_n; j++) {
        m_matrix[i][j] = entry;
      }
    }
  }

private:
  size_t m_rows_n;
  size_t m_columns_n;
  std::vector<std::vector<double>> m_matrix;
};

inline std::ostream & operator<<(std::ostream & outstr, const Matrix & matrix)
{
  for (size_t i = 0; i < matrix.rows(); i++) {
    outstr << "[";
    for (size_t j = 0; j < matrix.columns(); j++) {
      outstr << " " << matrix[i][j];
    }
    outstr << " ]" << std::endl;
  }

  return outstr;
}

#endif  // WOMBAT_CONTROL__DATA_TYPES__MATRIX_HPP_
