cmake_minimum_required(VERSION 3.5)
project(wombat_srrg)

find_package(wombat_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

message(DEBUG "${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION} found in (${OpenCV_DIR})")
message(DEBUG "${PROJECT_NAME}|OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

wombat_package()

add_subdirectory(srrg_hbst)
add_subdirectory(srrg2_core)
add_subdirectory(srrg2_solver)

wombat_package_install(
  EXPORT_DEPS
    Eigen3
    OpenCV
)
