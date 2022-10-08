cmake_minimum_required(VERSION 3.5)

project(wombat_cmake NONE)

include(cmake/WombatInstall.cmake)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_export_dependencies REQUIRED)

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
