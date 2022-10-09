#pragma once
#include "wombat_srrg/srrg_pcl/normal_computator.h"
#include "wombat_srrg/srrg_pcl/point_projector_lidar3d_types.h"
#include "wombat_srrg/srrg_pcl/point_projector_types.h"
#include "wombat_srrg/srrg_pcl/point_types_data.h"
#include "wombat_srrg/srrg_pcl/point_unprojector_lidar3d_types.h"
#include "wombat_srrg/srrg_pcl/point_unprojector_types.h"

namespace srrg2_core
{
  void point_cloud_registerTypes() __attribute__((constructor));
}
