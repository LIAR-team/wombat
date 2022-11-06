// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_config/configurable.h"
#include "wombat_srrg/srrg_config/preemptible.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_source_base.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_sink_base.h"
#include <mutex>
namespace srrg2_core {
  struct PipelineRunner: public MessageSinkBase, public Preemptible {
    PARAM(PropertyConfigurable_<MessageSourceBase>, source, "the source of the pipeline", nullptr, nullptr);
    void compute() override ;
    void reset() override;
    std::mutex& computeMutex() {return _compute_mutex;}
  protected:
    std::mutex _compute_mutex;
  };

  void srrg2_core_registerRunner() __attribute__((constructor));
}
