// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_messages/messages/pose_with_covariance_stamped_message.h"
namespace srrg2_core {
  PoseWithCovarianceStampedMessage::PoseWithCovarianceStampedMessage(const std::string& topic_,
                                                       const std::string& frame_id_,
                                                       const int& seq_,
                                                       const double& timestamp_):
    PoseStampedMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(covariance, Matrix6f::Identity()){}

  PoseWithCovarianceStampedMessage::~PoseWithCovarianceStampedMessage(){}
  
}
