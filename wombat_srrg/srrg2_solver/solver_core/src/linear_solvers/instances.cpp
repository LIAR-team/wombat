#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/instances.h"
#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/sparse_block_linear_solver_cholesky.h"
#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/sparse_block_linear_solver_cholesky_cholmod.h"
#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h"
#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/sparse_block_linear_solver_cholesky_diagonal.h"
#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/sparse_block_linear_solver_cholesky_emd.h"
#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/sparse_block_linear_solver_cholmod_full.h"
#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/sparse_block_linear_solver_ldl.h"

namespace srrg2_solver
{

void linear_solver_registerTypes()
{
  BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholeskyCSparse);
  BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholeskyCholmod);
  BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholmodFull);
  BOSS_REGISTER_CLASS(SparseBlockLinearSolverLDL);
  BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholesky);
  BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholeskyEMD);
  BOSS_REGISTER_CLASS(SparseBlockLinearSolverCholeskyDiagonal);
}

} // namespace srrg2_solver
