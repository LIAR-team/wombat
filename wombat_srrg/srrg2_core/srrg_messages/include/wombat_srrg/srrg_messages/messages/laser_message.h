// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_property/property_vector.h"
#include "wombat_srrg/srrg_messages/messages/base_sensor_message.h"

namespace srrg2_core {

  class LaserMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LaserMessage(const std::string& topic_ = "",
                 const std::string& frame_id_ = "",
                 const int& seq_ = -1,
                 const double& timestamp_ = -1.0);
    PropertyFloat angle_min;
    PropertyFloat angle_max;
    PropertyFloat angle_increment;
    PropertyFloat time_increment;
    PropertyFloat scan_time;
    PropertyFloat range_min;
    PropertyFloat range_max;
    PropertyVector_<float> ranges;
    PropertyVector_<float> intensities;
  };

  using LaserMessagePtr = std::shared_ptr<LaserMessage>;

}  // namespace srrg2_core
