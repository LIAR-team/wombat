cmake_minimum_required(VERSION 3.5)
project(wombat_control)

find_package(ament_cmake REQUIRED)
find_package(wombat_cmake REQUIRED)

wombat_package()

add_subdirectory(control)
add_subdirectory(data_types)
add_subdirectory(models)

if(BUILD_TESTING)
  wombat_linters()
  add_subdirectory(test)
endif()

ament_export_targets(wombat_controlTargets HAS_LIBRARY_TARGET)

ament_package()
