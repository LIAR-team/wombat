#include "wombat_srrg/srrg_solver/variables_and_factors/types_projective/variable_sim3_ad.h"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"
#include "wombat_srrg/srrg_solver/solver_core/variable.h"

namespace srrg2_solver
{

INSTANTIATE(VariableSim3EulerRightAD)
INSTANTIATE(VariableSim3EulerLeftAD)
INSTANTIATE(VariableSim3QuaternionRightAD)
INSTANTIATE(VariableSim3QuaternionLeftAD)

} // namespace srrg2_solver
