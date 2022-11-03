#include "differential_drive_odom_time_delay_sensor2d_error_factor_ad.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "wombat_srrg/srrg_solver/solver_core/ad_error_factor_impl.cpp"
#include "wombat_srrg/srrg_solver/solver_core/error_factor_impl.cpp"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  INSTANTIATE(DifferentialDriveOdomTimeDelaySensor2DErrorFactorAD)
} // namespace srrg2_solver
