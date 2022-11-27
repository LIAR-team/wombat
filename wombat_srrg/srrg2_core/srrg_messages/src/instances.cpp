// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "instances.h"
#include "wombat_srrg/srrg_messages/messages/camera_info_message.h"
#include "wombat_srrg/srrg_messages/messages/cmd_vel_message.h"
#include "wombat_srrg/srrg_messages/messages/grid_map_message.h"
#include "wombat_srrg/srrg_messages/messages/image_message.h"
#include "wombat_srrg/srrg_messages/messages/imu_message.h"
#include "wombat_srrg/srrg_messages/messages/joints_message.h"
#include "wombat_srrg/srrg_messages/messages/laser_message.h"
#include "wombat_srrg/srrg_messages/messages/navsat_fix_message.h"
#include "wombat_srrg/srrg_messages/messages/odometry_message.h"
#include "wombat_srrg/srrg_messages/messages/path_message.h"
#include "wombat_srrg/srrg_messages/messages/point_cloud2_message.h"
#include "wombat_srrg/srrg_messages/messages/point_stamped_message.h"
#include "wombat_srrg/srrg_messages/messages/pose_array_message.h"
#include "wombat_srrg/srrg_messages/messages/pose_message.h"
#include "wombat_srrg/srrg_messages/messages/pose_stamped_message.h"
#include "wombat_srrg/srrg_messages/messages/pose_with_covariance_stamped_message.h"
#include "wombat_srrg/srrg_messages/messages/range_message.h"
#include "wombat_srrg/srrg_messages/messages/ticks_message.h"
#include "wombat_srrg/srrg_messages/messages/transform_events_message.h"
#include "wombat_srrg/srrg_messages/messages/twist_stamped_message.h"
#include "wombat_srrg/srrg_messages/messages/planner_status_message.h"

#include "wombat_srrg/srrg_messages/message_handlers/message_file_source.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_odom_subsampler_source.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_pack.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_sorted_source.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_source_platform.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_synchronized_source.h"

#include "wombat_srrg/srrg_messages/message_handlers/message_file_sink.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_odom_subsampler_sink.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_platform_listener_sink.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_selector_sink.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_sorted_sink.h"
#include "wombat_srrg/srrg_messages/message_handlers/message_synchronized_sink.h"

namespace srrg2_core
{

void messages_registerTypes()
{
  BOSS_REGISTER_CLASS(PointCloud2DataBLOBReference); //, no idea if it should be CLASS or BLOB

  BOSS_REGISTER_CLASS(CameraInfoMessage);
  BOSS_REGISTER_CLASS(ImageMessage);
  BOSS_REGISTER_CLASS(IMUMessage);
  BOSS_REGISTER_CLASS(LaserMessage);
  BOSS_REGISTER_CLASS(OdometryMessage);
  BOSS_REGISTER_CLASS(PointStampedMessage);
  BOSS_REGISTER_CLASS(RangeMessage);
  BOSS_REGISTER_CLASS(TicksMessage);
  BOSS_REGISTER_CLASS(TransformEventsMessage);
  BOSS_REGISTER_CLASS(TwistStampedMessage);
  BOSS_REGISTER_CLASS(PointCloud2Message);
  BOSS_REGISTER_CLASS(JointsMessage);
  BOSS_REGISTER_CLASS(NavsatFixMessage)
  BOSS_REGISTER_CLASS(PoseMessage);
  BOSS_REGISTER_CLASS(PoseArrayMessage);
  BOSS_REGISTER_CLASS(PoseStampedMessage);
  BOSS_REGISTER_CLASS(PoseWithCovarianceStampedMessage);
  BOSS_REGISTER_CLASS(PathMessage);
  BOSS_REGISTER_CLASS(CmdVelMessage);
  BOSS_REGISTER_CLASS(GridMapMessage);
  BOSS_REGISTER_CLASS(PlannerStatusMessage);

  BOSS_REGISTER_CLASS(MessageFileSource);
  BOSS_REGISTER_CLASS(MessageSortedSource);
  BOSS_REGISTER_CLASS(MessageSynchronizedSource);
  BOSS_REGISTER_CLASS(MessageSourcePlatform);
  BOSS_REGISTER_CLASS(MessageOdomSubsamplerSource);

  BOSS_REGISTER_CLASS(MessagePack);
  BOSS_REGISTER_CLASS(MessageSelectorSink);
  BOSS_REGISTER_CLASS(MessageSinkBase);
  BOSS_REGISTER_CLASS(MessageFileSink);
  BOSS_REGISTER_CLASS(MessageSortedSink);
  BOSS_REGISTER_CLASS(MessageSynchronizedSink);
  BOSS_REGISTER_CLASS(MessageOdomSubsamplerSink);
  BOSS_REGISTER_CLASS(MessagePlatformListenerSink);
}

} // namespace srrg2_core
