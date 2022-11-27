#pragma once

#include <wombat_srrg/srrg_config/property_configurable.h>
#include <wombat_srrg/srrg_pcl/point_types.h>
#include <wombat_srrg/srrg2_slam_interfaces/mapping/scene_clipper.h>

namespace srrg2_laser_slam_2d
{

using SceneClipperPointNormal2f =
  srrg2_slam_interfaces::SceneClipper_<srrg2_core::Isometry2f,
                                        srrg2_core::PointNormal2fVectorCloud>;

using SceneClipperPointNormal2fPtr = std::shared_ptr<SceneClipperPointNormal2f>;

} // namespace srrg2_laser_slam_2d
