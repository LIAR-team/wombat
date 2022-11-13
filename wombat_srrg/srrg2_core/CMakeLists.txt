add_subdirectory(srrg_boss)
add_subdirectory(srrg_config)
add_subdirectory(srrg_data_structures)
add_subdirectory(srrg_geometry)
add_subdirectory(srrg_image)
add_subdirectory(srrg_matchable)
add_subdirectory(srrg_messages)
add_subdirectory(srrg_path_matrix)
add_subdirectory(srrg_pcl)
add_subdirectory(srrg_property)
add_subdirectory(srrg_system_utils)
add_subdirectory(srrg_test)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

#add_library(srrg2_core SHARED)
#add_library(wombat_srrg::srrg2_core ALIAS srrg2_core)
#target_link_libraries(srrg2_core PUBLIC wombat_srrg::srrg2_core::boss)


## Missing to build 

# add_subdirectory(apps)
# add_subdirectory(examples)
# add_subdirectory(misc_tests)
# add_subdirectory(srrg_benchmark)
# add_subdirectory(srrg_messages_viewers)
# add_subdirectory(srrg_viewer)


## Missing to complete

# srrg_messages
