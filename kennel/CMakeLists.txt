cmake_minimum_required(VERSION 3.5)
project(kennel)

find_package(wombat_cmake REQUIRED)
set(PKG_DEPENDENCIES
  geometry_msgs
  nav2_map_server
  rclcpp
  tf2
  tf2_geometry_msgs
)
foreach(PKG ${PKG_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()
find_package(wombat_core REQUIRED)

wombat_package()

add_library(kennel_lib SHARED)
target_sources(
  kennel_lib
  PRIVATE
    src/kennel.cpp
    src/diff-kinematic-model.cpp
    src/ground-truth-manager.cpp
    src/mobile-base.cpp
    src/robot-sim.cpp
    src/slam-manager.cpp
)
target_include_directories(
  kennel_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(kennel_lib PUBLIC ${PKG_DEPENDENCIES})
target_link_libraries(kennel_lib PUBLIC wombat_core::math)

add_executable(kennel main.cpp)
target_link_libraries(kennel PRIVATE kennel_lib)

wombat_install_target(kennel_lib)
wombat_install_target(kennel)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

wombat_package_install(
  EXPORT_DEPS
    ${PKG_DEPENDENCIES}
)
