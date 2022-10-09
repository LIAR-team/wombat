#pragma once
#include "wombat_srrg/srrg_messages/messages/base_sensor_message.h"
#include "wombat_srrg/srrg_data_structures/grid_map_2d.h"
#include "wombat_srrg/srrg_property/property_serializable.h"

namespace srrg2_core {
  class GridMapMessage : public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GridMapMessage(const std::string& topic_    = "",
                   const std::string& frame_id_ = "",
                   const int& seq_              = -1,
                   const double& timestamp_     = -1.0);
    PropertySerializable_<GridMap2D> grid_map_struct;
  };
  using GridMapMessagePtr = std::shared_ptr<GridMapMessage>;
} // namespace srrg2_core
