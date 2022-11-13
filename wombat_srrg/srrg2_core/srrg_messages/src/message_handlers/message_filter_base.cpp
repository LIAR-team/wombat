// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_messages/message_handlers/message_filter_base.h"
namespace srrg2_core {

  void MessageFilterBase::reset() {
    if (param_source.value()) {
      return param_source->reset();
    }
  }

  MessageSourceBase* MessageFilterBase::getRootSource() {
    if (param_source.value()) {
      return param_source->getRootSource();
    }
    return nullptr;
  }
} // namespace srrg2_core
