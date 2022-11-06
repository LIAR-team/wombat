// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_boss/blob.h"
#include "wombat_srrg/srrg_matchable/instances.h"

namespace srrg2_core {

  void matchable_registerTypes() {
    // ia TODO check this, should work
    BOSS_REGISTER_BLOB(MatchablefVectorData);
    BOSS_REGISTER_BLOB(MatchabledVectorData);
    BOSS_REGISTER_BLOB(VisualMatchablefVectorData);
    BOSS_REGISTER_BLOB(VisualMatchabledVectorData);
  }

} // namespace srrg2_core
