// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include "wombat_srrg/srrg_messages/messages/base_sensor_message.h"

namespace srrg2_core
{

  // a message pack "owns" its messages
  class MessagePack : public BaseSensorMessage {
  public:
    MessagePack(const std::string& topic_    = "",
                const std::string& frame_id_ = "",
                const int& seq_              = -1,
                const double& timestamp_     = -1.0);

    std::vector<BaseSensorMessagePtr> messages;

  };

  typedef std::shared_ptr<MessagePack> MessagePackPtr;
} // namespace srrg2_core
