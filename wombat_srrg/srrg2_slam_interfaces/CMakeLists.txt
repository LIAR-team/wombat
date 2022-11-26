
#ds enable HBST code for descriptor merging and opencv support TODO move to cmake_modules?
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSRRG_MERGE_DESCRIPTORS -DSRRG_HBST_HAS_OPENCV")


add_library(srrg2_slam_interfaces SHARED)
add_library(wombat_srrg::srrg2_slam_interfaces ALIAS srrg2_slam_interfaces)

target_include_directories(
  srrg2_slam_interfaces
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_sources(
  srrg2_slam_interfaces
  PRIVATE
    src/registration/aligner_slice_odometry_prior.cpp
    src/raw_data_preprocessors/raw_data_preprocessor_odom.cpp
    src/trackers/tracker_report.cpp
    src/trackers/tracker.cpp
    src/mapping/local_map.cpp
    src/system/multi_graph_slam_messages.cpp
    src/system/map_listener.cpp
    src/instances.cpp
)

target_link_libraries(
  srrg2_slam_interfaces
  PUBLIC
    wombat_srrg::hbst
    wombat_srrg::srrg2_core::benchmark
    #wombat_srrg::srrg2_core::srrg_messages::handlers
    wombat_srrg::srrg2_core::srrg_messages::messages
    wombat_srrg::srrg2_core::system_utils
    wombat_srrg::srrg2_solver::core
    wombat_srrg::srrg2_solver::utils
    wombat_srrg::srrg2_solver::types_2d
    wombat_srrg::srrg2_solver::types_3d
    ${OpenCV_LIBRARIES}
)

return()

add_executable(app_benchmark app_benchmark.cpp)
target_link_libraries(app_benchmark
  srrg2_slam_interfaces_library
  srrg2_config_library
  srrg2_messages_library
  ${catkin_LIBRARIES}
)


#wombat_install_target(srrg2_slam_interfaces)
