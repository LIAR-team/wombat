#include "instances.h"
#include "wombat_srrg/srrg_solver/solver_incremental/solver_incremental.h"
#include "wombat_srrg/srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"
#include "wombat_srrg/srrg_solver/solver_incremental/solver_incremental_runner.h"

namespace srrg2_solver
{

void solver_incremental_registerTypes()
{
  BOSS_REGISTER_CLASS(SolverIncremental);
  BOSS_REGISTER_CLASS(SolverIncrementalRunner);
  BOSS_REGISTER_CLASS(EndEpoch);
}

} // namespace srrg2_solver
