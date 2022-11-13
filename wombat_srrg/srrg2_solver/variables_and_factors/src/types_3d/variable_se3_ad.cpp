#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"
#include "wombat_srrg/srrg_solver/solver_core/variable.h"

namespace srrg2_solver {
  INSTANTIATE(VariableSE3EulerRightAD)
  INSTANTIATE(VariableSE3EulerLeftAD)
  INSTANTIATE(VariableSE3QuaternionRightAD)
  INSTANTIATE(VariableSE3QuaternionLeftAD)
  
} // namespace srrg2_solver
