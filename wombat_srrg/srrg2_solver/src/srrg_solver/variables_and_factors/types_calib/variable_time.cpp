#include "variable_time.h"
#include "wombat_srrg/srrg_solver/solver_core/variable_impl.cpp"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  VariableTime::~VariableTime(){}
  INSTANTIATE(VariableTime)
}
