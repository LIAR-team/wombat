cmake_minimum_required(VERSION 3.5)
project(wombat_control)

find_package(ament_cmake REQUIRED)
find_package(wombat_cmake REQUIRED)

wombat_package()

add_library(control SHARED)
add_library(wombat_control::control ALIAS control)

target_sources(
  control
  PRIVATE
  src/model/model_f_euler_dis.cpp
  src/control/position_controller.cpp
)

target_include_directories(
  control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if(BUILD_TESTING)
  wombat_linters()
  add_subdirectory(test)
endif()

install(
  DIRECTORY include/wombat_control
  DESTINATION include
)

install(
  TARGETS control
  EXPORT wombat_controlTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(wombat_controlTargets HAS_LIBRARY_TARGET)

ament_package()
