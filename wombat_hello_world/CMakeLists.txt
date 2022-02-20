cmake_minimum_required(VERSION 3.5)
project(wombat_hello_world)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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
ament_target_dependencies(hello_publisher PUBLIC rclcpp rclcpp_components std_msgs)

rclcpp_components_register_node(wombat_hello_world::hello_publisher
  PLUGIN "wombat_hello_world::HelloPublisher"
  EXECUTABLE hello_world
)

if(BUILD_TESTING)
  wombat_linters()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hello test/test_hello.cpp)
  target_link_libraries(test_hello wombat_hello_world::hello_publisher)
endif()

install(
  DIRECTORY include/wombat_hello_world
  DESTINATION include
)

install(
  TARGETS hello_publisher
  EXPORT wombat_hello_worldTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS hello_world
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(wombat_hello_worldTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs)

ament_package()
