// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include <iostream>

#include "wombat_srrg/srrg_matchable/matchable.h"
#include "wombat_srrg/srrg_property/vector_data.h"
#include "wombat_srrg/srrg_matchable/visual_matchable.h"

namespace srrg2_core {

  using MatchablefVectorData       = VectorData_<MatchablefVector>;
  using MatchabledVectorData       = VectorData_<MatchabledVector>;
  using VisualMatchablefVectorData = VectorData_<VisualMatchablefVector>;
  using VisualMatchabledVectorData = VectorData_<VisualMatchabledVector>;

  //! @brief serialization and instanciation of the types
  void matchable_registerTypes() __attribute__((constructor));

} // namespace srrg2_core
