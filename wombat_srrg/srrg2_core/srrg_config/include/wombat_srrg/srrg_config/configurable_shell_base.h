// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include "wombat_srrg/srrg_config/configurable.h"
#include "wombat_srrg/srrg_config/configurable_manager.h"
#include "wombat_srrg/srrg_config/preemptible.h"

namespace srrg2_core
{

class ConfigurableShellBase : public PreemptibleController
{
public:
  ConfigurableShellBase();

  virtual ~ConfigurableShellBase();
};

} // namespace srrg2_core
