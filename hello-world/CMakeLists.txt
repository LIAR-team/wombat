cmake_minimum_required(VERSION 3.5)
project(wombat_hello_world)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(wombat_cmake REQUIRED)

wombat_package()

add_executable(hello_world main.cpp)
ament_target_dependencies(hello_world rclcpp std_msgs)

install(TARGETS
  hello_world
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(wombat_linters REQUIRED)
  register_wombat_linters()
endif()

ament_package()
