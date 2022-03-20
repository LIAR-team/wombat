cmake_minimum_required(VERSION 3.5)

project(wombat_cmake NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_export_dependencies REQUIRED)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(DIRECTORY linters-config DESTINATION share/${PROJECT_NAME})

ament_package_xml()
ament_export_dependencies(${${PROJECT_NAME}_EXEC_DEPENDS})

ament_package(
  CONFIG_EXTRAS
    "WombatInstallTargetsExtras.cmake"
    "WombatLintersExtras.cmake"
    "WombatPackageExtras.cmake"
)
