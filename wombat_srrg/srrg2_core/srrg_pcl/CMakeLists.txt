add_library(srrg_pcl SHARED)
add_library(wombat_srrg::srrg2_core::srrg_pcl ALIAS srrg_pcl)

target_sources(
  srrg_pcl
  PRIVATE
    src/instances.cpp
    src/point_descriptor_field.cpp
    src/spacing.cpp
)

target_include_directories(
  srrg_pcl
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_pcl
  PUBLIC
    wombat_srrg::srrg2_core::srrg_boss
    wombat_srrg::srrg2_core::srrg_config
    wombat_srrg::srrg2_core::srrg_property
    wombat_srrg::srrg2_core::srrg_system_utils
    ${OpenCV_LIBS} # can be removed when cv::Mat descriptors are removed
)

wombat_install_target(srrg_pcl)
