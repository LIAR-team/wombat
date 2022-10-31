find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_costmap test_costmap.cpp)
target_link_libraries(test_costmap wombat_core::costmap)

ament_add_gtest(test_math_angles test_math_angles.cpp)
target_link_libraries(test_math_angles wombat_core::math)

ament_add_gtest(test_math_geometry_point test_math_geometry_point.cpp)
target_link_libraries(test_math_geometry_point wombat_core::math)

ament_add_gtest(test_math_utils test_math_utils.cpp)
target_link_libraries(test_math_utils wombat_core::math)
