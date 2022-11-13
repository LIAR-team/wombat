#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/imu_bias_variable.h"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"
#include "wombat_srrg/srrg_solver/solver_core/variable.h"
#include <wombat_srrg/srrg_geometry/geometry3d.h>

namespace srrg2_solver
{

using namespace srrg2_core;

void VariableImuBias::setZero() {
  this->_updated = true;
  _estimate.setZero();
}

void VariableImuBias::applyPerturbation(const Vector6f& pert) {
  this->_updated = true;
  _estimate += pert;
}

/*
void VariableImuBias::_drawImpl(ViewerCanvasPtr canvas_) const {
  throw std::runtime_error("VariableImuBias::draw| draw not implemented");
}
*/

void VariableImuBiasAD::applyPerturbationAD(const ADPerturbationVectorType& ad_pert) {
  _ad_estimate += ad_pert;
}

INSTANTIATE(VariableImuBias)
INSTANTIATE(VariableImuBiasAD)

} // namespace srrg2_solver
