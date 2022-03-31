find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_costmap test_costmap.cpp)
target_link_libraries(test_costmap wombat_ops::costmap)

ament_add_gtest(test_geometry test_geometry.cpp)
target_link_libraries(test_geometry wombat_ops::geometry)
