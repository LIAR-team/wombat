#include <gtest/gtest.h>

#include <wombat_srrg/srrg_system_utils/parse_command_line.h>
#include <wombat_srrg/srrg_system_utils/shell_colors.h>

// include solver stuff (instances)
#include "wombat_srrg/srrg_solver/solver_core/instances.h"
#include "wombat_srrg/srrg_solver/solver_core/solver.h"
// include types stuff (instances)

#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/all_types.h"

const std::string exe_name = "test_se3_motion_based_calib";
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_solver;

// global data
const size_t n_meas       = 100;
const size_t n_iterations = 20;

/* This function generates a fake relative 3d isometry */
static Isometry3f randomRelativeIso()
{
  const float tmax = 0.1;
  float x          = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float y          = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float z          = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float rx         = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float ry         = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  float rz         = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / tmax));
  Vector6f rand_pose;
  rand_pose << x, y, z, rx, ry, rz;
  Isometry3f rand_iso = srrg2_core::geometry3d::v2t(rand_pose);

  return rand_iso;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, SE3PoseMotionErrorFactorAD)
{
  using VariableType    = VariableSE3QuaternionRightAD;
  using VariablePtrType = std::shared_ptr<VariableType>;
  using FactorType      = SE3PoseMotionErrorFactorDataDriven;
  using FactorPtrType   = std::shared_ptr<FactorType>;
  using IsometryVector  = std::vector<Isometry3f, Eigen::aligned_allocator<Isometry3f>>;
  // generate dummy data
  const Vector6f sensor_in_robot_v     = 10 * Vector6f::Random();
  const Isometry3f sensor_in_robot     = geometry3d::ta2t(sensor_in_robot_v);
  const Isometry3f sensor_in_robot_inv = sensor_in_robot.inverse();

  IsometryVector measurements, relative_motions;
  CorrespondenceVector correspondences;

  measurements.reserve(n_meas);
  relative_motions.reserve(n_meas);
  correspondences.reserve(n_meas);
  // generating dataset
  for (size_t m = 0; m < n_meas; ++m) {
    Isometry3f rel_sensor_motion = randomRelativeIso();
    relative_motions.emplace_back(rel_sensor_motion);
    Isometry3f measurement = sensor_in_robot_inv * rel_sensor_motion * sensor_in_robot;
    measurements.emplace_back(measurement);
    correspondences.emplace_back(m, m);
  }

  VariablePtrType pose = VariablePtrType(new VariableType);
  pose->setEstimate(Isometry3f::Identity());
  pose->setGraphId(0);

  FactorPtrType factor = FactorPtrType(new FactorType);
  factor->setVariableId(0, 0);
  factor->setFixed(measurements);
  factor->setMoving(relative_motions);
  factor->setCorrespondences(correspondences);


  FactorGraphPtr graph(new FactorGraph);
  graph->addVariable(pose);
  graph->addFactor(factor);
  Solver solver;

  solver.param_max_iterations.pushBack(n_iterations);
  solver.param_termination_criteria.setValue(nullptr);
  solver.setGraph(graph);
  solver.compute();
  const auto& stats      = solver.iterationStats();
  const auto& final_chi2 = stats.back().chi_inliers;

  // assert performed iterations are the effectively n_iterations
  ASSERT_EQ(stats.size(), n_iterations);
  // assert chi2 is good
  ASSERT_LT(final_chi2, 1e-6);
  // assert that relative error is good
  const auto& estimated_T = pose->estimate();
  const auto diff_T       = estimated_T.inverse() * sensor_in_robot;
  const auto diff_vector  = geometry3d::t2tnq(diff_T);
  ASSERT_LT(diff_vector[0], 1e-5); // X
  ASSERT_LT(diff_vector[1], 1e-5); // Y
  ASSERT_LT(diff_vector[2], 1e-5); // Z
  ASSERT_LT(diff_vector[3], 1e-5); // qx
  ASSERT_LT(diff_vector[4], 1e-5); // qy
  ASSERT_LT(diff_vector[5], 1e-5); // qz
  LOG << stats << std::endl;
}
