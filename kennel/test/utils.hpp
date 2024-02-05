// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <gtest/gtest.h>

#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <string>

static std::filesystem::path s_data_dir;

inline void setup_data_dir_path()
{
  const auto env_var = "KENNEL_TEST_DATADIR";
  char * value = std::getenv(env_var);
  if (value == NULL) {
    std::cout << "The " << env_var << " environment variable is not set." << std::endl;
    assert(0);
  }

  s_data_dir = value;
}

inline std::string get_data_path(const std::string & filename)
{
  auto file_path = s_data_dir / filename;
  return file_path.string();
}
