
add_library(models SHARED)
add_library(wombat_control::models ALIAS models)

target_sources(
  models
  PRIVATE
    src/model_f_euler_dis.cpp
)

target_include_directories(
  models
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(
  models
  PUBLIC
    wombat_control::data_types
    wombat_msgs::data_types
)

wombat_install_target(models)
