cmake_minimum_required(VERSION 3.5)

project(wombat_cmake NONE)

find_package(ament_cmake REQUIRED)

# The scripts are meant to be used by packages that depend on this one,
# so they assume that the following CMake variable is set (this is set when this package is found).
set(wombat_cmake_DIR cmake)

include(cmake/InstallHooks.cmake)
include(cmake/WombatInstall.cmake)
include(cmake/WombatLinters.cmake)
include(cmake/WombatPackage.cmake)

wombat_package()

run_install_githooks_script()

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
      "WombatTargetExtras.cmake"
)
