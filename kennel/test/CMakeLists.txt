find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_sim_time_manager test_sim_time_manager.cpp)
target_link_libraries(test_sim_time_manager kennel::kennel rclcpp::rclcpp)
