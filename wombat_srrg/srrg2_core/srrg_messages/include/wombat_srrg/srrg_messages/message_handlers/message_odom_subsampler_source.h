#pragma once
#include "wombat_srrg/srrg_messages/message_handlers/message_filter_base.h"
#include "wombat_srrg/srrg_config/configurable.h"
#include "wombat_srrg/srrg_config/param_macros.h"

namespace srrg2_core {

  class MessageOdomSubsamplerSource : public MessageFilterBase {
  public:
    PARAM(PropertyString, odom_topic, "odometry topic to subsample", "/odom", 0);
    PARAM(PropertyFloat, translation_min, "minimum translation between odometries [m]", 0.25, 0);
    PARAM(PropertyFloat, rotation_min, "minimum rotation between odometries [rad]", 0.25, 0);

  public:
    BaseSensorMessagePtr getMessage() override;

  protected:
    float _cum_rotation    = 0; // cumulative rotation since last output message
    float _cum_translation = 0; // cumulative translation since last output message
    Isometry3f _prev_pose  = Isometry3f::Identity();
  };

} // namespace srrg2_core
