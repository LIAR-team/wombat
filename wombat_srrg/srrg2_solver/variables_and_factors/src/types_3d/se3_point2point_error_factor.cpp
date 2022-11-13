#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h"
#include "wombat_srrg/srrg_solver/solver_core/ad_error_factor_impl.hpp"
#include "wombat_srrg/srrg_solver/solver_core/error_factor_impl.hpp"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver
{

using namespace srrg2_core;

void SE3Point2PointErrorFactor::errorAndJacobian(bool error_only)
{
  const Isometry3f& X_    = _variables.template at<0>()->estimate();
  const Vector3f& moving_ = *(this->moving);
  const Vector3f& fixed_  = *(this->fixed);
  const Matrix3f& R      = X_.linear();
  _e                     = X_ * moving_ - fixed_;
  if (error_only) {
    return;
  }
  _J.block<3, 3>(0, 0) = R;
  _J.block<3, 3>(0, 3) = -2.f * R * geometry3d::skew(moving_);
}

INSTANTIATE(SE3Point2PointErrorFactor)
INSTANTIATE(SE3Point2PointErrorFactorCorrespondenceDriven)

void SE3Point2PointWithSensorErrorFactor::errorAndJacobian(bool error_only)
{
  const Isometry3f& X_    = _robot_in_sensor * _variables.template at<0>()->estimate();
  const Vector3f& moving_ = *(this->moving);
  const Vector3f& fixed_  = *(this->fixed);
  const Matrix3f& R      = X_.linear();
  _e                     = X_ * moving_ - fixed_;
  if (error_only) {
    return;
  }
  _J.block<3, 3>(0, 0) = R;
  _J.block<3, 3>(0, 3) = -2.f * R * geometry3d::skew(moving_);
}

INSTANTIATE(SE3Point2PointWithSensorErrorFactor)
INSTANTIATE(SE3Point2PointWithSensorErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver