#include "gps_factor_ad.h"
#include <wombat_srrg/srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <wombat_srrg/srrg_solver/solver_core/error_factor_impl.cpp>
#include <wombat_srrg/srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver {

  void GpsErrorFactorAD::_drawImpl(ViewerCanvasPtr canvas_) const {
  }

  INSTANTIATE(GpsErrorFactorAD)
} // namespace srrg2_solver
