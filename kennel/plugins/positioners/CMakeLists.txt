
wombat_add_library(kennel_positioner_plugins SHARED)
target_sources(
  kennel_positioner_plugins
  PRIVATE
    stub_positioner.cpp
)
target_link_libraries(
  kennel_positioner_plugins
  PRIVATE
    kennel::common
)
ament_target_dependencies(kennel_positioner_plugins PUBLIC pluginlib)
wombat_install_target(kennel_positioner_plugins)
