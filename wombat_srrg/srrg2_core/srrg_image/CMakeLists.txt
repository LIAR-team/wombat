add_library(srrg_image SHARED)
add_library(wombat_srrg::srrg2_core::srrg_image ALIAS srrg_image)

target_sources(
  srrg_image
  PRIVATE
    src/image.cpp
    src/image_data.cpp
)

target_include_directories(
  srrg_image
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_image
  PUBLIC
    wombat_srrg::srrg2_core::data_structures
    wombat_srrg::srrg2_core::boss
    ${OpenCV_LIBS}
)

wombat_install_target(srrg_image)
  