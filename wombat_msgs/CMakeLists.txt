cmake_minimum_required(VERSION 3.5)
project(wombat_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(wombat_cmake REQUIRED)

find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

wombat_package(NO_WOMBAT_LINTERS)

# Generate ROS 2 interfaces and create wombat_msgs library
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Explore.action"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

# Create data_types library
add_library(data_types INTERFACE)
add_library(wombat_msgs::data_types ALIAS data_types)

target_include_directories(
  data_types
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

wombat_install_target(data_types)

wombat_package_install(
  EXPORT_DEPS
    rosidl_default_runtime
)
