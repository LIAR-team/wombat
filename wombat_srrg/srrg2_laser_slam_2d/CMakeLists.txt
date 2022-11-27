
###########
## srrg2_laser_slam_2d_mapping library
############

add_library(srrg2_laser_slam_2d_mapping SHARED)
add_library(wombat_srrg::srrg2_laser_slam_2d::mapping ALIAS srrg2_laser_slam_2d_mapping)

target_include_directories(
  srrg2_laser_slam_2d_mapping
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_sources(
  srrg2_laser_slam_2d_mapping
  PRIVATE
    src/mapping/grid_mapper_2d.cpp
    src/mapping/merger_projective_2d.cpp
    src/mapping/scene_clipper_ball_2d.cpp
    src/mapping/scene_clipper_projective_2d.cpp
)

target_link_libraries(
  srrg2_laser_slam_2d_mapping
  PUBLIC
    wombat_srrg::srrg2_core::config
    wombat_srrg::srrg2_core::data_structures
    wombat_srrg::srrg2_core::geometry
    wombat_srrg::srrg2_core::messages
    wombat_srrg::srrg2_core::pcl
    wombat_srrg::srrg2_core::system_utils
    wombat_srrg::srrg2_slam_interfaces
    wombat_srrg::srrg2_solver::types_2d
)

wombat_install_target(srrg2_laser_slam_2d_mapping)

###########
## srrg2_laser_slam_2d_registration library
############

add_library(srrg2_laser_slam_2d_registration SHARED)
add_library(wombat_srrg::srrg2_laser_slam_2d::registration ALIAS srrg2_laser_slam_2d_registration)

target_include_directories(
  srrg2_laser_slam_2d_registration
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_sources(
  srrg2_laser_slam_2d_registration
  PRIVATE
    src/registration/aligner_slice_processor_laser_2d.cpp
    src/registration/correspondence_finder_kd_tree_2d.cpp
    src/registration/correspondence_finder_nn_2d.cpp
    src/registration/correspondence_finder_projective_2d.cpp
)

target_link_libraries(
  srrg2_laser_slam_2d_registration
  PUBLIC
    wombat_srrg::srrg2_core::config
    wombat_srrg::srrg2_core::data_structures
    wombat_srrg::srrg2_core::path_matrix
    wombat_srrg::srrg2_core::pcl
    wombat_srrg::srrg2_slam_interfaces
    wombat_srrg::srrg2_solver::types_2d
)

wombat_install_target(srrg2_laser_slam_2d_registration)

###########
## srrg2_laser_slam_2d_sensor_processing library
############

add_library(srrg2_laser_slam_2d_sensor_processing SHARED)
add_library(wombat_srrg::srrg2_laser_slam_2d::sensor_processing ALIAS srrg2_laser_slam_2d_sensor_processing)

target_include_directories(
  srrg2_laser_slam_2d_sensor_processing
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_sources(
  srrg2_laser_slam_2d_sensor_processing
  PRIVATE
    src/sensor_processing/raw_data_preprocessor_projective_2d.cpp
)

target_link_libraries(
  srrg2_laser_slam_2d_sensor_processing
  PUBLIC
    wombat_srrg::srrg2_core::config
    wombat_srrg::srrg2_core::data_structures
    wombat_srrg::srrg2_core::geometry
    wombat_srrg::srrg2_core::messages
    wombat_srrg::srrg2_core::pcl
    wombat_srrg::srrg2_slam_interfaces
)

wombat_install_target(srrg2_laser_slam_2d_sensor_processing)

###########
## srrg2_laser_slam_2d library
############

add_library(srrg2_laser_slam_2d SHARED)
add_library(wombat_srrg::srrg2_laser_slam_2d ALIAS srrg2_laser_slam_2d)

target_include_directories(
  srrg2_laser_slam_2d
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_sources(
  srrg2_laser_slam_2d
  PRIVATE
    src/instances.cpp
)

target_link_libraries(
  srrg2_laser_slam_2d
  PUBLIC
    wombat_srrg::srrg2_core::boss
    wombat_srrg::srrg2_core::path_matrix
    wombat_srrg::srrg2_laser_slam_2d::mapping
    wombat_srrg::srrg2_laser_slam_2d::registration
    wombat_srrg::srrg2_laser_slam_2d::sensor_processing
    wombat_srrg::srrg2_slam_interfaces
    wombat_srrg::srrg2_solver::core
    wombat_srrg::srrg2_solver::linear_solvers
    wombat_srrg::srrg2_solver::types_2d
    wombat_srrg::srrg2_solver::utils

)

wombat_install_target(srrg2_laser_slam_2d)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

return()










return()

add_executable(app_benchmark src/apps/app_benchmark.cpp)
target_link_libraries(app_benchmark
  wombat_srrg::srrg2_slam_interfaces
  wombat_srrg::srrg2_core::config
  wombat_srrg::srrg2_core::srrg_messages::messages
)




target_link_libraries(srrg2_laser_slam_2d_library
  srrg2_laser_slam_2d_sensor_processing_library
  srrg2_laser_slam_2d_mapping_library
  srrg2_laser_slam_2d_registration_library
  ${catkin_LIBRARIES}
)








project(srrg2_laser_slam_2d)

find_package(catkin REQUIRED COMPONENTS
  tf
  sensor_msgs
  srrg_cmake_modules
  srrg2_core
  srrg2_core_ros
  srrg2_solver
#  srrg2_qgl_viewport
  srrg2_slam_interfaces
)


find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_laser_slam_2d_library
    srrg2_laser_slam_2d_registration_library
    srrg2_laser_slam_2d_sensor_processing_library
    srrg2_laser_slam_2d_mapping_library
  CATKIN_DEPENDS
    srrg2_slam_interfaces
    srrg2_core
    srrg2_core_ros
    srrg2_solver
    srrg2_qgl_viewport
    sensor_msgs
    tf
    srrg_cmake_modules

)

#ds includes provided by this package (restrictive compile flags will be applied!)
include_directories(src)

#ds specify additional locations of header files
#ds treating them as system includes to suppress a massive wave of warnings originating from those header files (!)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

# set sources
add_subdirectory(src)

# if (SRRG_COMPILE_APPS)
#   add_subdirectory(apps)
# endif()

#ds gtest directory
if (SRRG_COMPILE_TESTS)
  add_subdirectory(tests)
endif()

#ds regression testing
if (SRRG_COMPILE_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()
