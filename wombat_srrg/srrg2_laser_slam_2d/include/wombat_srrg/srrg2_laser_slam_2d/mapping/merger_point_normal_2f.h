#pragma once

#include <wombat_srrg/srrg_config/configurable.h>
#include <wombat_srrg/srrg_geometry/geometry_defs.h>
#include <wombat_srrg/srrg_pcl/point_types.h>
#include <wombat_srrg/srrg2_slam_interfaces/mapping/merger.h>

namespace srrg2_laser_slam_2d
{

using MergerPointNormal2f = srrg2_slam_interfaces::Merger_<srrg2_core::Isometry2f,
                                                            srrg2_core::PointNormal2fVectorCloud,
                                                            srrg2_core::PointNormal2fVectorCloud>;

using MergerPointNormal2fPtr = std::shared_ptr<MergerPointNormal2f>;

} // namespace srrg2_laser_slam_2d
