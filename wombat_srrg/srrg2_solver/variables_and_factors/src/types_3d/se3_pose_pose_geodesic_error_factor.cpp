#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_derivatives_helpers.h"
#include "wombat_srrg/srrg_solver/solver_core/error_factor_impl.hpp"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"
#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/variable_se3.h"

namespace srrg2_solver
{

void SE3PosePoseGeodesicErrorFactor::errorAndJacobian(bool error_only_)
{
  // take objects - static cast for speeeeeed
  VariableSE3QuaternionRight* v_from = _variables.at<0>();
  VariableSE3QuaternionRight* v_to   = _variables.at<1>();
  assert(v_from && v_to &&
          "SE3PosePoseGeodesicErrorFactor::errorAndJacobian|invalid variables type");

  const auto& from_T      = v_from->estimate();
  const auto& to_T        = v_to->estimate();
  const auto prediction_T = from_T.inverse() * to_T;

  // compute the error as translation and normalized quaternion
  const auto error_T = _inverse_measurement * prediction_T;
  _e                 = geometry3d::t2tnq(error_T);

  if (error_only_) {
    return;
  }

  // compute analitic jacobians using super evil stuff that has been wisely hidden in this
  // helper class that you don't want to open. never open it. never.
  srrg2_core::Matrix6_<Scalar> Ji, Jj;
  SE3PosePoseGeodesicErrorFactorJacobianHelper<Scalar>::computeGeodesicJacobians(
    _inverse_measurement, prediction_T, error_T, Ji, Jj);

  // set this goddamn jacobian
  jacobian<0>() = Ji;
  jacobian<1>() = Jj;
}

/*
void SE3PosePoseGeodesicErrorFactor::_drawImpl(ViewerCanvasPtr canvas_) const
{
  if (!canvas_) {
    throw std::runtime_error("SE3PosePoseGeodesicQuaternionErrorFactor::draw|invalid canvas");
  }
  Vector3f coords[2];
  coords[0] =
    reinterpret_cast<const VariableSE3QuaternionRight*>(variable(0))->estimate().translation();
  coords[1] =
    reinterpret_cast<const VariableSE3QuaternionRight*>(variable(1))->estimate().translation();

  float lw=0.5;
  if (fabs(variableId(0)-variableId(1))==1) {
    lw*=2;
  }
  lw*=(level()*3+1);
  canvas_->pushColor();
  canvas_->pushLineWidth();
  canvas_->setLineWidth(lw);
  float fading=1.-0.5*level();
  Vector3f color=srrg2_core::ColorPalette::color3fBlue()*fading;
  canvas_->setColor(color);
  canvas_->putLine(2, coords);
  canvas_->popAttribute();
  canvas_->popAttribute();
}
*/

INSTANTIATE(SE3PosePoseGeodesicErrorFactor)

} // namespace srrg2_solver
