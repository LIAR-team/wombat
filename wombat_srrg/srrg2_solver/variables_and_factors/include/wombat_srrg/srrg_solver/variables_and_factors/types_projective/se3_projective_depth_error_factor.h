#pragma once
#include "wombat_srrg/srrg_solver/solver_core/ad_error_factor.h"
#include "wombat_srrg/srrg_solver/solver_core/factor_correspondence_driven.h"
#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/variable_se3.h"
#include "wombat_srrg/srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include <wombat_srrg/srrg_geometry/geometry3d.h>
#include <wombat_srrg/srrg_pcl/point_types.h>

namespace srrg2_solver
{

using namespace srrg2_core;

/***** point to point *****/

class SE3ProjectiveDepthErrorFactor : public ErrorFactor_<3, VariableSE3QuaternionRight>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using BaseType   = ErrorFactor_<3, VariableSE3QuaternionRight>;
  using FixedType  = Point3f;
  using MovingType = Point3f;

  inline void setFixed(const Point3f & fixed)
  {
    _fixed_point = &fixed.coordinates();
  }
  inline void setMoving(const Point3f & moving)
  {
    _moving_point = &moving.coordinates();
  }

  void errorAndJacobian(bool error_only = false) override;

  inline void setCameraMatrix(const srrg2_core::Matrix3f & camera_matrix)
  {
    _camera_matrix = camera_matrix;
  }

  inline void setImageDim(const srrg2_core::Vector2f & image_dim)
  {
    _image_dim = image_dim;
  }

protected:
  const Vector3f* _moving_point       = nullptr;
  const Vector3f* _fixed_point        = nullptr;
  srrg2_core::Matrix3f _camera_matrix = srrg2_core::Matrix3f::Identity();
  srrg2_core::Vector2f _image_dim     = srrg2_core::Vector2f::Zero();
};

// correspondence factor
using SE3ProjectiveDepthErrorFactorCorrespondenceDriven =
  FactorCorrespondenceDriven_<SE3ProjectiveDepthErrorFactor,
                              Point3fVectorCloud,
                              Point3fVectorCloud>;

class SE3ProjectiveDepthWithSensorErrorFactor : public SE3ProjectiveDepthErrorFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using BaseType     = SE3ProjectiveDepthErrorFactor;
  using FixedType    = Point3f;
  using MovingType   = Point3f;
  using EstimateType = Isometry3f;

  void errorAndJacobian(bool error_only = false) final;

  inline void setSensorInRobot(const EstimateType & sensor_in_robot)
  {
    _robot_in_sensor = sensor_in_robot.inverse();
  }

protected:
  EstimateType _robot_in_sensor = Isometry3f::Identity();
};

// correspondence factor
using SE3ProjectiveDepthWithSensorErrorFactorCorrespondenceDriven =
  FactorCorrespondenceDriven_<SE3ProjectiveDepthWithSensorErrorFactor,
                              Point3fVectorCloud,
                              Point3fVectorCloud>;

} // namespace srrg2_solver
