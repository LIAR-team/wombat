#include "wombat_srrg/srrg2_slam_interfaces/registration/aligners/aligner_slice_odometry_prior.h"
#include "wombat_srrg/srrg2_slam_interfaces/registration/aligners/aligner_slice_processor_prior.h"

namespace srrg2_slam_interfaces
{

void AlignerSliceOdom2DPrior::setupFactor()
{
  EstimateType delta = EstimateType::Identity();
  if (_count > 1) {
    delta = _fixed_slice->inverse() * (*_moving_slice);
  }
  _factor->setMeasurement(delta);
  _factor->setInformationMatrix(
    param_diagonal_info_matrix.value().asDiagonal()); //< hack // TODO explain hack
}

void AlignerSliceOdom2DPrior::init(AlignerType * aligner)
{
  BaseType::init(aligner);
  setupFactor();
  _aligner->setMovingInFixed(_factor->measurement());
  ++_count;
}

void AlignerSliceOdom3DPrior::setupFactor()
{
  EstimateType delta = EstimateType::Identity();
  if (_count > 1) {
    delta = _fixed_slice->inverse() * (*_moving_slice);
  }
  _factor->setMeasurement(delta);
}

void AlignerSliceOdom3DPrior::init(AlignerType * aligner)
{
  BaseType::init(aligner);
  setupFactor();
  _aligner->setMovingInFixed(_factor->measurement());
  _factor->setInformationMatrix(param_diagonal_info_matrix.value().asDiagonal());
  ++_count;
}

} // namespace srrg2_slam_interfaces
