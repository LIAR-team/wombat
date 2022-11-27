#include "wombat_srrg/srrg2_laser_slam_2d/registration/aligner_slice_processor_laser_2d.h"

#include <wombat_srrg/srrg2_slam_interfaces/registration/aligners/aligner_slice_processor.h>
#include <wombat_srrg/srrg2_slam_interfaces/registration/aligners/aligner_termination_criteria.h>

namespace srrg2_laser_slam_2d
{

void AlignerSliceProcessorLaser2DWithSensor::setupFactor()
{
  BaseType::setupFactor();
  BaseType::setupFactorWithSensor("MultiAlignerLaser2DWithSensorSlice");
}

} // namespace srrg2_laser_slam_2d
