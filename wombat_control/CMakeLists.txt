cmake_minimum_required(VERSION 3.5)
project(wombat_control)

find_package(wombat_cmake REQUIRED)
find_package(wombat_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

wombat_package()

add_subdirectory(control)
add_subdirectory(data_types)
add_subdirectory(models)

if(BUILD_TESTING)
  wombat_linters()
  add_subdirectory(test)
endif()

wombat_package_install(
  EXPORT_DEPS
    Eigen3
)
