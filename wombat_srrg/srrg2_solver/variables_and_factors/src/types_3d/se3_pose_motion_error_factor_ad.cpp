#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/se3_pose_motion_error_factor_ad.h"
#include "wombat_srrg/srrg_solver/solver_core/ad_error_factor_impl.hpp"
#include "wombat_srrg/srrg_solver/solver_core/error_factor_impl.hpp"
#include "wombat_srrg/srrg_solver/solver_core/instance_macros.h"

namespace srrg2_solver {
  INSTANTIATE(SE3PoseMotionErrorFactorAD)
  INSTANTIATE(SE3PoseMotionErrorFactorDataDriven)
}
