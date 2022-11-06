// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_messages/message_handlers/message_pack.h"

namespace srrg2_core {

  MessagePack::MessagePack(const std::string& topic_,
                           const std::string& frame_id_,
                           const int& seq_,
                           const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_) {
  }

} // namespace srrg2_core
