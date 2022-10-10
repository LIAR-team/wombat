// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_config/configurable.h"
#include "wombat_srrg/srrg_property/property_identifiable.h"

namespace srrg2_core {

  template <typename ContainerType_>
  using PropertyConfigurable_ = PropertyIdentifiablePtr_<std::shared_ptr<ContainerType_>>;

  template <typename ContainerType_>
  using PropertyConfigurableNoOwnership_ = PropertyIdentifiablePtr_<std::weak_ptr<ContainerType_>>;
} // namespace srrg2_core
