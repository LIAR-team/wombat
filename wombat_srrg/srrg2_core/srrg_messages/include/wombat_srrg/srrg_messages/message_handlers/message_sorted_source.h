// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_messages/message_handlers/message_filter_base.h"

namespace srrg2_core {

  class MessageSortedSource : public MessageFilterBase {
  public:
    PARAM(PropertyDouble, time_interval, "lag time to sort messages", 1., 0);

    MessageSortedSource();
    BaseSensorMessagePtr getMessage() override;

    int numDroppedMessages() const {
      return _num_dropped_messages;
    }

    void resetCounters();
    void reset() override;

  protected:
    double earliestStamp();
    inline double latestStamp();
    std::multimap<double, BaseSensorMessagePtr> _msg_queue;
    int _num_dropped_messages = 0;
  };

  using MessageSortedSourcePtr = std::shared_ptr<MessageSortedSource>;

} // namespace srrg2_core
