#pragma once
#include "solver_stats.h"
#include "wombat_srrg/srrg_solver/solver_core/sparse_block_matrix/matrix_block.h"
#include "variable_ptr_tuple.h"
#include <wombat_srrg/srrg_data_structures/correspondence.h>
#include <wombat_srrg/srrg_data_structures/iterator_interface.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  struct RobustifierBase;

  /*! @brief Base class that represents of factor.
   */
  class FactorDrawable :public FactorBase, public DrawableBase
  {
    //void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };

} // namespace srrg2_solver
