// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include <string>
#include <iostream>
#include <memory>
#include "wombat_srrg/srrg_property/property_container.h"
#include "wombat_srrg/srrg_geometry/geometry_defs.h"
#include "wombat_srrg/srrg_geometry/geometry3d.h"

namespace srrg2_core {

  class BaseSensorMessage: public PropertyContainerSerializable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BaseSensorMessage(const std::string& topic_ = "",
                      const std::string& frame_id_ = "",
                      int seq_ = -1,
                      const double& timestamp_ = -1);

    PropertyString topic;
    PropertyString frame_id;
    PropertyInt seq;
    PropertyDouble timestamp;
  };

  typedef std::shared_ptr<BaseSensorMessage> BaseSensorMessagePtr;
  typedef std::vector<BaseSensorMessagePtr> BaseSensorMessagePtrVector;

} /* namespace srrg2_core */

