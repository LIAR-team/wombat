cmake_minimum_required(VERSION 3.5)
project(wombat_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(wombat_cmake REQUIRED)

find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

wombat_package(AUTO_GENERATED_CODE)

# Generate ROS 2 interfaces and create wombat_msgs library
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Explore.action"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

wombat_package_install(
  NO_EXPORT_TARGETS
  EXPORT_DEPS
    rosidl_default_runtime
)
