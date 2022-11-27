add_library(srrg_test INTERFACE)
add_library(wombat_srrg::srrg2_core::test ALIAS srrg_test)

target_include_directories(
  srrg_test
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

wombat_install_target(srrg_test)
