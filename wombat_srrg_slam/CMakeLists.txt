cmake_minimum_required(VERSION 3.5)
project(wombat_srrg_slam)

find_package(wombat_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

wombat_package()

add_subdirectory(hbst)
#add_subdirectory(srrg2_core)
#add_subdirectory(srrg2_solver)

wombat_package_install(
  EXPORT_DEPS
    Eigen3
    OpenCV
)
