find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_matrix test_matrix.cpp)
target_link_libraries(test_matrix wombat_control::data_types)

ament_add_gtest(test_model test_model.cpp)
target_link_libraries(test_model wombat_control::models)

ament_add_gtest(test_position_controller test_position_controller.cpp)
target_link_libraries(test_position_controller wombat_control::control wombat_control::models)
