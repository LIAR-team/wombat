add_subdirectory(viewer_core)

add_library(srrg2_viewer_library SHARED
  viewer_core_base.cpp viewer_core_base.h
  drawable_base.cpp drawable_base.h
  active_drawable.cpp active_drawable.h
  viewer_canvas.cpp viewer_canvas.h
  viewer_context_base.cpp viewer_context_base.h
  viewer_context_shared.cpp viewer_context_shared.h
  viewer_manager_base.cpp viewer_manager_base.h
  viewer_manager_shared.cpp viewer_manager_shared.h
)

target_link_libraries(srrg2_viewer_library
  srrg2_viewer_core_library
  srrg2_system_utils_library
)


#### moved here configurable shell!

return()

add_library(srrg2_configurable_shell_library SHARED
pipeline_runner.cpp
pipeline_runner.h

configurable_shell.cpp
configurable_shell.h

linenoise.c
linenoise.h
)

target_link_libraries(
srrg2_configurable_shell_library
srrg2_property_library
srrg2_messages_library
srrg2_viewer_library
srrg2_config_library
srrg2_boss_library
)

add_executable(preemptible_example preemptible_example.cpp)

target_link_libraries(preemptible_example
srrg2_configurable_shell_library
${catkin_LIBRARIES}
)