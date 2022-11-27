#include "wombat_srrg/srrg2_laser_slam_2d/instances.h"
#include <wombat_srrg/srrg_benchmark/slam_benchmark_suite_carmen.hpp>
#include <wombat_srrg/srrg_pcl/instances.h>
#include <wombat_srrg/srrg2_slam_interfaces/instances.h>

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;

#ifdef CURRENT_SOURCE_DIR
const std::string current_source_directory = CURRENT_SOURCE_DIR;
#else
const std::string current_source_directory = "./";
#endif

const Vector2f maximum_mean_translation_rmse               = Vector2f(25.0, 25.0);
const Vector2f maximum_standard_deviation_translation_rmse = Vector2f(100.0, 100.0);
const Vector1f maximum_mean_rotation_rmse                  = Vector1f(1.0);
const Vector1f maximum_standard_deviation_rotation_rmse    = Vector1f(1.0);

int main(int argc_, char** argv_) {
  srrg2_core::messages_registerTypes();
  srrg2_core::point_cloud_registerTypes();
  srrg2_laser_slam_2d::srrg2_laser_slam_2d_registerTypes();
  Profiler::enable_logging = true;

  // load a laser slam assembly from configuration
  ConfigurableManager manager;
  manager.read(current_source_directory + "/../../configs/killian.conf");
  MultiGraphSLAM2DPtr slammer = manager.getByName<MultiGraphSLAM2D>("slam");
  assert(slammer);

  // instantiate the benchmark utility for AIS
  SLAMBenchmarkSuiteSE2Ptr benchamin(new SLAMBenchmarkSuiteCARMEN());

  // load complete dataset and ground truth from disk
  benchamin->loadDataset("mit_killian_court.json");
  benchamin->loadGroundTruth("mit_killian_court_gt.txt");

  // process all messages and feed benchamin with computed estimates
  // TODO clearly this does not account for PGO/BA which happens retroactively
  while (BaseSensorMessagePtr message = benchamin->getMessage()) {
    SystemUsageCounter::tic();
    slammer->setRawData(message);
    slammer->compute();
    const double processing_duration_seconds = SystemUsageCounter::toc();
    benchamin->setPoseEstimate(
      slammer->robotInWorld(), message->timestamp.value(), processing_duration_seconds);
  }

  // run benchmark evaluation
  benchamin->compute();

  // save trajectory for external benchmark plot generation
  benchamin->writeTrajectoryToFile("trajectory_ais_killian.txt");

  // evaluate if target metrics have not been met
  if (benchamin->isRegression(maximum_mean_translation_rmse,
                              maximum_standard_deviation_translation_rmse,
                              maximum_mean_rotation_rmse,
                              maximum_standard_deviation_rotation_rmse)) {
    return -1;
  } else {
    return 0;
  }
}
