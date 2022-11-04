#pragma once
#include "wombat_srrg/srrg_solver/solver_core/error_factor.h"
#include "wombat_srrg/srrg_solver/variables_and_factors/types_2d/variable_point2.h"
#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/variable_se3.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  /**
   * @brief Bundle adjustment + calibartion 3D factor for Pose-Point-Offset estimation.
   * Error is computed by referring the point in the world frame (inverse pose of the sensor in
   * world) and computing the difference with the measure
   */
 
  class SE3PosePointBAErrorFactor : public ErrorFactor_<2,
                                                        VariableSE3QuaternionRight, // robot pose
                                                        VariablePoint3,             // point in world
                                                        VariableVector4,            // fx, fy, cx, cy
                                                        VariableSE3QuaternionRight  // camera_in_robot
                                                        >,
                                        public MeasurementOwnerEigen_<Vector2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void errorAndJacobian(bool error_only_ = false) final;
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };
} // namespace srrg2_solver
