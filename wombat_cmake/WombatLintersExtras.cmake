# copied from ament_lint_auto/ament_lint_auto-extras.cmake

if(_${PROJECT_NAME}_AMENT_PACKAGE)
  message(FATAL_ERROR "find_package(wombat_cmake) must be called before ament_package()")
endif()

find_package(ament_cmake_core QUIET REQUIRED)
find_package(ament_cmake_test QUIET REQUIRED)

include("${wombat_cmake_DIR}/WombatLinters.cmake")
