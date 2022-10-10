// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_config/property_configurable.h"

namespace srrg2_core {

  template <typename ContainerType_>
  using PropertyConfigurableVector_ =
    PropertyIdentifiablePtrVector_<std::shared_ptr<ContainerType_>>;
}
