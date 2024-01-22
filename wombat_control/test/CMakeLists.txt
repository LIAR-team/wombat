find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_model test_model.cpp)
target_link_libraries(test_model tf2_geometry_msgs::tf2_geometry_msgs wombat_core::math wombat_control::models)

ament_add_gtest(test_position_controller test_position_controller.cpp)
target_link_libraries(test_position_controller wombat_control::control wombat_control::models)
