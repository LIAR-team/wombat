// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_messages/messages/pose_message.h"
namespace srrg2_core {
  PoseMessage::PoseMessage():
    SETUP_PROPERTY(pose_vector, Vector6f::Zero()){
  }

  PoseMessage::~PoseMessage(){}

}
