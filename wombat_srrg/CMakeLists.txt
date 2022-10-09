cmake_minimum_required(VERSION 3.5)
project(wombat_srrg)

find_package(wombat_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

wombat_package()

add_subdirectory(srrg_hbst)

wombat_package_install(
  EXPORT_DEPS
    Eigen3
)
