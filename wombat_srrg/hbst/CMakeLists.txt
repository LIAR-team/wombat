add_library(srrg_hbst INTERFACE)
add_library(wombat_srrg::hbst ALIAS srrg_hbst)

target_include_directories(
  srrg_hbst
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_hbst
  INTERFACE
    Eigen3::Eigen
)

wombat_install_target(srrg_hbst)

#ds HBST generic compilation flags - ADD THEM IN YOUR PROJECT AS WELL TO ENABLE THEM (HBST is header-only)
# - SRRG_MERGE_DESCRIPTORS: HBST checks for identical descriptors stemming from multiple images and represents them with a single entity
#two descriptors are merged if their distance is within a certain threshold
#the threshold can be controlled with the static variable maximum_distance_for_merge
#a value of maximum_distance_for_merge=0 means they have to be identical
#the higher this value, the more compact and efficient becomes the tree at the cost of matching accuracy
#this flag is recommended for large-scale datasets and lifelong recognition applications

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_search tests/test_search.cpp)
  target_link_libraries(test_search wombat_srrg::hbst)
  ament_add_gtest(test_streaming tests/test_streaming.cpp)
  target_link_libraries(test_streaming wombat_srrg::hbst)

  # target_compile_definitions(test_streaming_merging PRIVATE SRRG_MERGE_DESCRIPTORS)
endif()
