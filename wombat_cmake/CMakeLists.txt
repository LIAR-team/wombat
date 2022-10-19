cmake_minimum_required(VERSION 3.5)

project(wombat_cmake NONE)

find_package(ament_cmake REQUIRED)

# The scripts are meant to be used by packages that depend on this one,
# so they assume that the following CMake variable is set (this is set when this package is found).
set(wombat_cmake_DIR cmake)

include(cmake/WombatInstall.cmake)
include(cmake/WombatPackage.cmake)

wombat_package()

# Add a configure dependency on the githook files to detect when they are changed
set_property(
  DIRECTORY
  APPEND
  PROPERTY
    CMAKE_CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/githooks/install_hooks.sh
    ${CMAKE_SOURCE_DIR}/githooks/pre-push.sh
)
# Install githooks as part of this package configuration
execute_process(
  COMMAND ${CMAKE_SOURCE_DIR}/githooks/install_hooks.sh
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  RESULT_VARIABLE install_hooks_ret
)
if(install_hooks_ret)
  message(FATAL_ERROR "Failed to install git hook: '${install_hooks_ret}'")
endif()

if(BUILD_TESTING)
  include(cmake/WombatLinters.cmake)
  wombat_linters()
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(DIRECTORY linters-config DESTINATION share/${PROJECT_NAME})

wombat_package_install(
  NO_EXPORT_TARGETS
  EXPORT_DEPS
    ${${PROJECT_NAME}_EXEC_DEPENDS}
    ament_cmake
  AMENT_PACKAGE
    CONFIG_EXTRAS
      "WombatInstallExtras.cmake"
      "WombatLintersExtras.cmake"
      "WombatPackageExtras.cmake"
)
