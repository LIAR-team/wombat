
add_library(srrg_messages_messages STATIC)
add_library(wombat_srrg::srrg2_core::srrg_messages::messages ALIAS srrg_messages_messages)

target_sources(
  srrg_messages_messages
  PRIVATE
    src/messages/base_sensor_message.cpp
    src/messages/camera_info_message.cpp
    src/messages/cmd_vel_message.cpp
    src/messages/grid_map_message.cpp
    src/messages/image_message.cpp
    src/messages/imu_message.cpp
    src/messages/joints_message.cpp
    src/messages/laser_message.cpp
    src/messages/navsat_fix_message.cpp
    src/messages/odometry_message.cpp
    src/messages/path_message.cpp
    src/messages/planner_status_message.cpp
    #src/messages/point_cloud2_message.cpp
    src/messages/point_field.cpp
    src/messages/point_stamped_message.cpp
    src/messages/pose_array_message.cpp
    src/messages/pose_message.cpp
    src/messages/pose_stamped_message.cpp
    src/messages/pose_with_covariance_stamped_message.cpp
    src/messages/range_message.cpp
    src/messages/ticks_message.cpp
    src/messages/transform_events_message.cpp
    src/messages/twist_stamped_message.cpp
)

target_include_directories(
  srrg_messages_messages
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_messages_messages
  PUBLIC
    wombat_srrg::srrg2_core::data_structures
    wombat_srrg::srrg2_core::srrg_image
    wombat_srrg::srrg2_core::property
    wombat_srrg::srrg2_core::system_utils
)

wombat_install_target(srrg_messages_messages)

return()

add_library(srrg_messages_handlers SHARED)
add_library(wombat_srrg::srrg2_core::srrg_messages::handlers ALIAS srrg_messages_handlers)

target_sources(
  srrg_messages_handlers
  PRIVATE
    src/message_handlers/message_file_sink.cpp
    src/message_handlers/message_file_sink_base.cpp
    src/message_handlers/message_filter_base.cpp
    src/message_handlers/message_odom_subsampler_sink.cpp
    src/message_handlers/message_file_source.cpp
    src/message_handlers/message_file_source_base.cpp
    src/message_handlers/message_odom_subsampler_source.cpp
    src/message_handlers/message_pack.cpp
    src/message_handlers/message_platform_listener_sink.cpp
    src/message_handlers/message_sink_base.cpp
    src/message_handlers/message_selector_sink.cpp
    src/message_handlers/message_sorted_source.cpp
    src/message_handlers/message_source_base.cpp
    src/message_handlers/message_source_platform.cpp
    src/message_handlers/message_sorted_sink.cpp
    src/message_handlers/message_synchronized_sink.cpp
    src/message_handlers/message_synchronized_source.cpp
)

target_include_directories(
  srrg_messages_handlers
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_messages_handlers
  PUBLIC
    wombat_srrg::srrg2_core::config
    wombat_srrg::srrg2_core::data_structures
    wombat_srrg::srrg2_core::srrg_image
    wombat_srrg::srrg2_core::srrg_messages::messages
    wombat_srrg::srrg2_core::property
    wombat_srrg::srrg2_core::system_utils
)

wombat_install_target(srrg_messages_handlers)

add_library(srrg_messages SHARED)
add_library(wombat_srrg::srrg2_core::srrg_messages ALIAS srrg_messages)

target_sources(
  srrg_messages
  PRIVATE
    src/instances.cpp
)

target_link_libraries(
  srrg_messages
  PUBLIC
    wombat_srrg::srrg2_core::srrg_boss
    wombat_srrg::srrg2_core::srrg_messages::handlers
    wombat_srrg::srrg2_core::srrg_messages::messages
)

wombat_install_target(srrg_messages)
