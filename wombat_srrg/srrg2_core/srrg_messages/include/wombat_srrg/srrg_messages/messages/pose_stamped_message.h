#pragma once
#include "wombat_srrg/srrg_messages/messages/base_sensor_message.h"
#include "wombat_srrg/srrg_messages/messages/pose_message.h"
#include "wombat_srrg/srrg_property/property_serializable.h"
#include "wombat_srrg/srrg_geometry/geometry_defs.h"
#include "wombat_srrg/srrg_property/property_eigen.h"

namespace srrg2_core {
  class PoseStampedMessage: public BaseSensorMessage {
  public:
    PoseStampedMessage(const std::string& topic_    = "",
                       const std::string& frame_id_ = "",
                       const int& seq_              = -1,
                       const double& timestamp_     = -1);

    PropertySerializable_<PoseMessage> pose;
  };

  using PoseStampedMessagePtr = std::shared_ptr<PoseStampedMessage>;
}
