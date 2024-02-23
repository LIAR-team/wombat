find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_sim_time_manager test_sim_time_manager.cpp)
target_link_libraries(test_sim_time_manager kennel::kennel_gtest rclcpp::rclcpp)

ament_add_gtest(test_collisions test_collisions.cpp
  ENV KENNEL_TEST_DATADIR=${CMAKE_CURRENT_SOURCE_DIR}/data)
target_link_libraries(test_collisions kennel::kennel_gtest)

ament_add_gtest(test_kennel_api test_kennel_api.cpp
  ENV KENNEL_TEST_DATADIR=${CMAKE_CURRENT_SOURCE_DIR}/data)
target_link_libraries(test_kennel_api kennel::kennel_gtest rclcpp::rclcpp)

ament_add_gtest(test_empty_world test_empty_world.cpp
  ENV KENNEL_TEST_DATADIR=${CMAKE_CURRENT_SOURCE_DIR}/data)
target_link_libraries(test_empty_world kennel::kennel_gtest)

ament_add_gtest(test_range_projection test_range_projection.cpp
  ENV KENNEL_TEST_DATADIR=${CMAKE_CURRENT_SOURCE_DIR}/data)
target_link_libraries(test_range_projection kennel::kennel_gtest)

ament_add_gtest(test_walls_world test_walls_world.cpp
  ENV KENNEL_TEST_DATADIR=${CMAKE_CURRENT_SOURCE_DIR}/data)
target_link_libraries(test_walls_world kennel::kennel_gtest)
