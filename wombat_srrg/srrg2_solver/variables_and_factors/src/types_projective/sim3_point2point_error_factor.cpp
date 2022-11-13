#include "wombat_srrg/srrg_solver/variables_and_factors/types_projective/sim3_point2point_error_factor.h"
#include "wombat_srrg/srrg_solver/solver_core/error_factor_impl.hpp"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver
{
using namespace srrg2_core;

void Sim3Point2PointErrorFactor::errorAndJacobian(bool error_only)
{
  const Similiarity3f& X_ = _variables.template at<0>()->estimate();
  const Vector3f& moving_ = *(this->moving);
  const Vector3f& fixed_  = *(this->fixed);
  const Matrix3f& R      = X_.linear();
  const float s          = 1.f / X_.inverseScaling();
  _e                     = X_ * moving_ - fixed_;
  if (error_only) {
    return;
  }

  _J.block<3, 3>(0, 0) = s * R;
  _J.block<3, 3>(0, 3) = -2.f * s * R * geometry3d::skew(moving_);
  _J.block<3, 1>(0, 6) = -s * R * moving_;
}

INSTANTIATE(Sim3Point2PointErrorFactor)
INSTANTIATE(Sim3Point2PointErrorFactorCorrespondenceDriven)

} // namespace srrg2_solver
