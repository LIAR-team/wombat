#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/sparse_block_linear_solver_cholesky_emd.h"
#include "wombat_srrg/srrg_solver/solver_core/linear_solvers/quotient_graph.h"

namespace srrg2_solver {
  void SparseBlockLinearSolverCholeskyEMD ::computeOrderingHint(
    std::vector<int>& ordering,
    const std::vector<IntPair>& layout) const {
    const int dim = ordering.size();
    QuotientGraph emd_graph(layout, dim);
    emd_graph.setPolicy(QuotientGraph::Policy::Approximate);
    emd_graph.mdo(ordering);
  }

} // namespace srrg2_solver
