// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_messages/messages/base_sensor_message.h"
#include "wombat_srrg/srrg_property/property.h"

namespace srrg2_core {

  class CmdVelMessage : public BaseSensorMessage {
  public:
    CmdVelMessage(const std::string& topic_    = "",
               const std::string& frame_id_ = "",
               const int& seq_              = -1,
               const double& timestamp_     = -1);
    PropertyFloat linear;
    PropertyFloat angular;
  };

  using CmdVelMessagePtr = std::shared_ptr<CmdVelMessage>;

} // namespace srrg2_core