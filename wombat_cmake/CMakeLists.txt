cmake_minimum_required(VERSION 3.5)

project(wombat_cmake NONE)

find_package(ament_cmake REQUIRED)

# The scripts are meant to be used by packages that depend on this one,
# so they assume that the following CMake variable is set (this is set when this package is found).
set(wombat_cmake_DIR cmake)

include(cmake/InstallHooks.cmake)
include(cmake/WombatConfigExtras.cmake)
include(cmake/WombatInstall.cmake)
include(cmake/WombatPackage.cmake)

wombat_package()

run_install_githooks_script()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(DIRECTORY linters-config DESTINATION share/${PROJECT_NAME})

set(CONFIG_EXTRA_NAME "${CMAKE_CURRENT_BINARY_DIR}/WombatCmakeConfigExtras.cmake")
set(CMAKE_SCRIPTS_TO_INSTALL
  WombatConfigExtras.cmake
  WombatInstall.cmake
  WombatLinters.cmake
  WombatPackage.cmake
  WombatTarget.cmake
)
wombat_write_config_extras(
  OUTPUT_PATH
    ${CONFIG_EXTRA_NAME}
  SCRIPT_NAMES
    ${CMAKE_SCRIPTS_TO_INSTALL}
)

wombat_package_install(
  NO_EXPORT_TARGETS
  EXPORT_DEPS
    ${${PROJECT_NAME}_EXEC_DEPENDS}
    ament_cmake
  AMENT_PACKAGE
    CONFIG_EXTRAS
      ${CONFIG_EXTRA_NAME}
)
