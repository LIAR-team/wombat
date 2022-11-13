#include "differential_drive_odom_error_factor_ad.h"
#include "wombat_srrg/srrg_solver/solver_core/ad_error_factor_impl.hpp"
#include "wombat_srrg/srrg_solver/solver_core/error_factor_impl.hpp"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  INSTANTIATE(DifferentialDriveOdomErrorFactorAD)
}
