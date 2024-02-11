cmake_minimum_required(VERSION 3.5)
project(wombat_hello_world)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wombat_cmake REQUIRED)

wombat_package()

wombat_add_library(hello_publisher SHARED)
target_sources(hello_publisher PRIVATE src/hello_publisher.cpp)
target_include_directories(
  hello_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(hello_publisher PUBLIC rclcpp::rclcpp)
ament_target_dependencies(hello_publisher PUBLIC std_msgs)

add_executable(hello_world src/main.cpp)
target_link_libraries(hello_world PRIVATE wombat_hello_world::hello_publisher)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hello test/test_hello.cpp)
  target_link_libraries(test_hello wombat_hello_world::hello_publisher)
endif()

wombat_install_target(hello_publisher)
wombat_install_target(hello_world)

wombat_package_install(
  EXPORT_DEPS
    rclcpp std_msgs
)
