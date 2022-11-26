// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include <dirent.h>
#include <gtest/gtest.h>
#include <random>

#include "wombat_srrg/srrg_geometry/geometry_defs.h"

// helper macros aimed to extend gtest macros
#define ASSERT_IN_RANGE(VALUE, MIN, MAX) \
  ASSERT_GE(VALUE, MIN);                 \
  ASSERT_LE(VALUE, MAX)

#define ASSERT_NOTNULL(POINTER) ASSERT_NE(POINTER, nullptr)
#define ASSERT_NULL(POINTER) ASSERT_EQ(POINTER, nullptr)

#define ASSERT_EQ_VECTOR3F(VALUE, VALUE_REFERENCE)                                       \
  std::cerr << "ASSERT_EQ_VECTOR3F|deprecated, change to: ASSERT_EQ_EIGEN" << std::endl; \
  ASSERT_EQ(VALUE(0), VALUE_REFERENCE(0));                                               \
  ASSERT_EQ(VALUE(1), VALUE_REFERENCE(1));                                               \
  ASSERT_EQ(VALUE(2), VALUE_REFERENCE(2))

#define ASSERT_EQ_EIGEN(VALUE_, VALUE_REFERENCE_)                        \
  ASSERT_EQ(VALUE_.rows(), VALUE_REFERENCE_.rows());                     \
  ASSERT_EQ(VALUE_.cols(), VALUE_REFERENCE_.cols());                     \
  for (decltype(VALUE_.rows()) __r = 0; __r < VALUE_.rows(); ++__r) {          \
    for (decltype(VALUE_.cols()) __c = 0; __c < VALUE_.cols(); ++__c) {        \
      ASSERT_EQ(VALUE_.matrix()(__r, __c), VALUE_REFERENCE_.matrix()(__r, __c)); \
    }                                                                    \
  }

#define ASSERT_NEAR_VECTOR3F(VALUE, VALUE_REFERENCE, TOLERANCE)                              \
  std::cerr << "ASSERT_NEAR_VECTOR3F|deprecated, change to: ASSERT_NEAR_EIGEN" << std::endl; \
  ASSERT_NEAR(VALUE(0), VALUE_REFERENCE(0), TOLERANCE);                                      \
  ASSERT_NEAR(VALUE(1), VALUE_REFERENCE(1), TOLERANCE);                                      \
  ASSERT_NEAR(VALUE(2), VALUE_REFERENCE(2), TOLERANCE)

#define ASSERT_NEAR_EIGEN(VALUE_, VALUE_REFERENCE_, TOLERANCE_)                        \
  ASSERT_EQ(VALUE_.rows(), VALUE_REFERENCE_.rows());                                   \
  ASSERT_EQ(VALUE_.cols(), VALUE_REFERENCE_.cols());                                   \
  for (decltype(VALUE_.rows()) __r = 0; __r < VALUE_.rows(); ++__r) {                        \
    for (decltype(VALUE_.cols()) __c = 0; __c < VALUE_.cols(); ++__c) {                      \
      ASSERT_NEAR(VALUE_.matrix()(__r, __c), VALUE_REFERENCE_.matrix()(__r, __c), TOLERANCE_); \
    }                                                                                  \
  }

#define ASSERT_LT_ABS(VALUE, VALUE_REFERENCE) ASSERT_LT(std::fabs(VALUE), VALUE_REFERENCE)

//! unittest namespace, not to be used in any other context!
namespace srrg2_test
{

//! current user folder used for test execution
static std::string filepath_user_folder = "";

//! current srrg source folder used for test execution
static std::string filepath_folder_srrg = "";

//! current srrg2 source folder used for test execution
static std::string filepath_folder_srrg2 = "";

//! flag that is set to true if test data path is set correctly
static bool is_test_data_available = false;

//! parse current user folder from CLI parameters
static void parseUserFolderUNIX(const std::string & filepath)
{
  // determine local srrg test data path
  // TODO remove this mother of all evil
  const size_t index_file_root_slash = filepath.find_first_of("/");
  if (index_file_root_slash == std::string::npos) {
    std::cerr << "parseUserFolder|ERROR: unable to infer file root (/) from CLI" << std::endl;
    return;
  }
  const size_t index_root_folder_slash = filepath.find_first_of("/", index_file_root_slash + 1);
  if (index_root_folder_slash == std::string::npos) {
    std::cerr << "parseUserFolder|WARNING: unable to infer home "
                  "folder (/root, /home) from CLI"
              << std::endl;
    return;
  }
  const size_t index_home_folder_slash =
    filepath.find_first_of("/", index_root_folder_slash + 1);
  if (index_home_folder_slash == std::string::npos) {
    std::cerr << "parseUserFolder|WARNING: unable to infer user folder(/ "
                  "home / user) from CLI "
              << std::endl;
    return;
  }

  // assemble user folder (might be invalid at this point)
  filepath_user_folder = filepath.substr(0, index_home_folder_slash);

  // check if we are in the root folder (CI case) if home is not present
  if (filepath_user_folder.find("home") == std::string::npos) {
    std::cerr << "parseUserFolder|WARNING: home folder not found - assuming root" << std::endl;
    filepath_user_folder = filepath.substr(0, index_root_folder_slash);
  }
  filepath_folder_srrg  = filepath_user_folder + "/source/srrg/";
  filepath_folder_srrg2 = filepath_user_folder + "/source/srrg2/";

  // check if the derived folders do not exist
  if (opendir(filepath_folder_srrg.c_str()) == nullptr) {
    std::cerr << "parseUserFolder|WARNING: srrg folder not existing: '" + filepath_folder_srrg +
                    "'"
              << std::endl;
    return;
  }
  if (opendir(filepath_folder_srrg2.c_str()) == nullptr) {
    std::cerr << "parseUserFolder|WARNING: srrg2 folder not existing: '" + filepath_folder_srrg2 +
                    "'"
              << std::endl;
    return;
  }

  // fine if still here
  std::cerr << "parseUserFolderUNIX|using folder SRRG: '" << filepath_folder_srrg << "'"
            << std::endl;
  std::cerr << "parseUserFolderUNIX|using folder SRRG2: '" << filepath_folder_srrg2 << "'"
            << std::endl;
  is_test_data_available = true;
}

//! test entry points
static int runTests(int argc_, char** argv_, const bool& use_test_folder_ = false)
{
  if (use_test_folder_) {
    parseUserFolderUNIX(argv_[0]);
  }
  testing::InitGoogleTest(&argc_, argv_);
  return RUN_ALL_TESTS();
}

template <typename RealType_ = float>
class RandomNumberGeneratorBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RandomNumberGeneratorBase() {
    // constant seed for reproducible tests
    _random_device = std::mt19937(0);
  }
  virtual ~RandomNumberGeneratorBase() {
  }
  virtual RealType_ getRandomScalar(const RealType_& mean,
                                    const RealType_& standard_deviation) = 0;

protected:
  // random number generation
  std::mt19937 _random_device;
};

template <typename RealType_ = float>
class RandomNumberGeneratorUniform : public RandomNumberGeneratorBase<RealType_>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RandomNumberGeneratorUniform()
  {
    // constant seed for reproducible tests
    srand(0);
  }

  virtual ~RandomNumberGeneratorUniform() = default;

  virtual RealType_ getRandomScalar(
    const RealType_& mean,
    const RealType_& standard_deviation) override
  {
    const RealType_ minimum(mean - standard_deviation);
    const RealType_ maximum(mean + standard_deviation);
    std::uniform_real_distribution<RealType_> distribution(minimum, maximum);
    return distribution(this->_random_device);
  }

  template <int Dimension_>
  srrg2_core::Vector_<RealType_, Dimension_>
  getRandomVector(
    const srrg2_core::Vector_<RealType_, Dimension_>& mean,
    const srrg2_core::Vector_<RealType_, Dimension_>& standard_deviation)
  {
    const srrg2_core::Vector_<RealType_, Dimension_> minimum(mean - standard_deviation);
    const srrg2_core::Vector_<RealType_, Dimension_> maximum(mean + standard_deviation);
    srrg2_core::Vector_<RealType_, Dimension_> random_vector;
    for (int i = 0; i < Dimension_; ++i) {
      std::uniform_real_distribution<RealType_> distribution(minimum(i), maximum(i));
      random_vector(i) = distribution(this->_random_device);
    }
    return random_vector;
  }
};

template <typename RealType_ = float>
class RandomNumberGeneratorGaussian : public RandomNumberGeneratorBase<RealType_>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~RandomNumberGeneratorGaussian() = default;

  virtual RealType_ getRandomScalar(
    const RealType_ & mean,
    const RealType_ & standard_deviation) override
  {
    std::normal_distribution<RealType_> distribution(mean, standard_deviation);
    return distribution(this->_random_device);
  }

  template <int Dimension_>
  srrg2_core::Vector_<RealType_, Dimension_>
  getRandomVector(
    const srrg2_core::Vector_<RealType_, Dimension_> & mean,
    const srrg2_core::Vector_<RealType_, Dimension_> & standard_deviation)
  {
    srrg2_core::Vector_<RealType_, Dimension_> random_vector;
    for (int i = 0; i < Dimension_; ++i) {
      std::normal_distribution<RealType_> distribution(mean(i), standard_deviation(i));
      random_vector(i) = distribution(this->_random_device);
    }
    return random_vector;
  }
};

} // namespace srrg2_test
