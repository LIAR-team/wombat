find_package(ament_cmake_gtest REQUIRED)

### COSTMAP

ament_add_gtest(test_costmap test_costmap.cpp)
target_link_libraries(test_costmap wombat_core::costmap)

### GRID

ament_add_gtest(test_grid_boundary_projection test_grid_boundary_projection.cpp)
target_link_libraries(test_grid_boundary_projection wombat_core::grid)

ament_add_gtest(test_grid_coordinates test_grid_coordinates.cpp)
target_link_libraries(test_grid_coordinates wombat_core::grid)

ament_add_gtest(test_grid_geometry test_grid_geometry.cpp)
target_link_libraries(test_grid_geometry wombat_core::grid)

ament_add_gtest(test_grid_neighbors test_grid_neighbors.cpp)
target_link_libraries(test_grid_neighbors wombat_core::grid)

ament_add_gtest(test_grid_raytrace test_grid_raytrace.cpp)
target_link_libraries(test_grid_raytrace wombat_core::grid)

### MATH

ament_add_gtest(test_math_angles test_math_angles.cpp)
target_link_libraries(test_math_angles wombat_core::math)

ament_add_gtest(test_math_geometry_point test_math_geometry_point.cpp)
target_link_libraries(test_math_geometry_point wombat_core::math)

ament_add_gtest(test_math_online_stats test_math_online_stats.cpp)
target_link_libraries(test_math_online_stats wombat_core::math)

ament_add_gtest(test_math_utils test_math_utils.cpp)
target_link_libraries(test_math_utils wombat_core::math)

### ROS 2

ament_add_gtest(test_ros2_parameters test_ros2_parameters.cpp)
target_link_libraries(test_ros2_parameters wombat_core::ros2)
