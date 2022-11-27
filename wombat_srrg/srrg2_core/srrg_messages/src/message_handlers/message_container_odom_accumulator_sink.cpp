// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_messages/message_handlers/message_container_odom_accumulator_sink.h"
#include "wombat_srrg/srrg_messages/messages/odometry_message.h"

namespace srrg2_core
{

bool MessageContainerOdomAccumulatorSink::putMessage(BaseSensorMessagePtr msg)
{
  if (!_container) {
    return false;
  }
  if (_topic != msg->topic.value()) {
    return false;
  }
  if (auto odom_ptr = std::dynamic_pointer_cast<OdometryMessage>(msg)) {
    double timestamp_seconds = odom_ptr->timestamp.value();
    const Isometry3f& pose   = odom_ptr->pose.value();
    _container->insert(std::make_pair(timestamp_seconds, pose));
  }
  return true;
}

} // namespace srrg2_core
