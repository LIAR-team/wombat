cmake_minimum_required(VERSION 3.5)
project(wombat_hello_world)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wombat_cmake REQUIRED)

wombat_package()

add_library(hello_publisher SHARED)
add_library(wombat_hello_world::hello_publisher ALIAS hello_publisher)
target_sources(hello_publisher PRIVATE src/hello_publisher.cpp)
target_include_directories(
  hello_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(hello_publisher PUBLIC rclcpp std_msgs)

add_executable(hello_world src/main.cpp)
target_link_libraries(hello_world PRIVATE wombat_hello_world::hello_publisher)

if(BUILD_TESTING)
  wombat_linters()

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
