add_library(srrg_property SHARED)
add_library(wombat_srrg::srrg2_core::property ALIAS srrg_property)

target_sources(
  srrg_property
  PRIVATE
    src/property.cpp
    src/property_eigen.cpp
    src/property_identifiable.cpp
    src/property_container.cpp
    src/property_container_manager.cpp
)

target_include_directories(
  srrg_property
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_property
  PUBLIC
    wombat_srrg::srrg2_core::srrg_boss
    wombat_srrg::srrg2_core::system_utils
    Eigen3::Eigen
)

wombat_install_target(srrg_property)
