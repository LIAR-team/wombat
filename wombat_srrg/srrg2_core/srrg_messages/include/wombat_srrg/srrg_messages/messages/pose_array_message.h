// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_messages/messages/base_sensor_message.h"
#include "wombat_srrg/srrg_messages/messages/pose_message.h"
#include "wombat_srrg/srrg_property/property_serializable.h"
#include "wombat_srrg/srrg_geometry/geometry_defs.h"

namespace srrg2_core {
  class PoseArrayMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseArrayMessage(const std::string& topic_    = "",
                    const std::string& frame_id_ = "",
                    const int& seq_              = -1,
                    const double& timestamp_     = -1);

    PropertySerializableVector_<PoseMessage> poses;

    virtual ~PoseArrayMessage();

  protected:


  };

  using PoseArrayMessagePtr = std::shared_ptr<PoseArrayMessage>;
}
