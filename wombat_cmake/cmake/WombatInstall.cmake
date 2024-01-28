# Copyright 2022 Soragna Alberto.
# All Rights Reserved.
# Unauthorized copying via any medium is strictly prohibited.
# Proprietary and confidential.

include(CMakeParseArguments)

#
# Private function to install an executable CMake target.
# Users should call `wombat_install_target`
#
# @private
#
function(__wombat_install_executable_target TARGET_NAME)
  # Install the executable
  install(TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

#
# Private function to install a library CMake target.
# Users should call `wombat_install_target`
#
# @private
#
function(__wombat_install_library_target TARGET_NAME)
  # Include directories are usually PUBLIC, but we need to make them INTERFACE
  # for INTERFACE libraries.
  get_target_property(type ${TARGET_NAME} TYPE)

  set(include_keyword PUBLIC)
  if(${type} STREQUAL "INTERFACE_LIBRARY")
    set(include_keyword INTERFACE)
  endif()
  # Specify install-time include directories
  target_include_directories(
    ${TARGET_NAME}
    ${include_keyword}
      $<INSTALL_INTERFACE:${TARGET_NAME}/include>
  )
  # Install include directories
  set(include_dir include/${PROJECT_NAME})
  if(EXISTS ${include_dir})
    install(
      DIRECTORY ${include_dir}
      DESTINATION ${TARGET_NAME}/include
    )
  endif()
  # Install the library target and associate it with the export target
  install(
    TARGETS ${TARGET_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION ${TARGET_NAME}/include
  )
endfunction()

#
# Installs a CMake target and optionally also its include directories.
# The target is associated with a project-specific export.
# The export is NOT installed by this function: see `wombat_install_export_targets`
#
# Note that, in order to use this function, it is necessary to:
# - add `<buildtool_depend>wombat_cmake</buildtool_depend>` to your project package.xml file
# - add `find_package(wombat_cmake REQUIRED)` to your project main CMakeLists.txt
#
# @public
#
function(wombat_install_target TARGET_NAME)

  if(NOT TARGET ${TARGET_NAME})
    message(FATAL_ERROR "Target ${TARGET_NAME} passed to wombat_install_target does not exist")
  endif()

  # Find out if the target is an executable or a library
  get_target_property(type ${TARGET_NAME} TYPE)

  if(${type} STREQUAL "EXECUTABLE")
    __wombat_install_executable_target(${TARGET_NAME})
  else()
    __wombat_install_library_target(${TARGET_NAME})
  endif()
endfunction()

#
# Installs a project-specific export target.
# Library targets are associated to this export when installed using
# `wombat_install_target`
#
# This macro must be called from a top-level CMakeLists.txt, not from a subdirectory!
# See https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html#building-a-library
#
# Note that, in order to use this function, it is necessary to:
# - add `<buildtool_depend>wombat_cmake</buildtool_depend>` to your project package.xml file
# - add `find_package(wombat_cmake REQUIRED)` to your project main CMakeLists.txt
#
# @public
#
macro(wombat_install_export_targets)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
endmacro()

#
# Standard utility to install a whole CMake package.
#
# This macro must be called from a top-level CMakeLists.txt, not from a subdirectory!
# See https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html#building-a-library
#
# Note that, in order to use this function, it is necessary to:
# - add `<buildtool_depend>wombat_cmake</buildtool_depend>` to your project package.xml file
# - add `find_package(wombat_cmake REQUIRED)` to your project main CMakeLists.txt
#
# @public
#
macro(wombat_package_install)
  cmake_parse_arguments(
    ARG # prefix of output variables
    "NO_EXPORT_TARGETS" # list of names of the boolean arguments (only defined ones will be true)
    "" # list of names of mono-valued arguments
    "EXPORT_DEPS;AMENT_PACKAGE" # list of names of multi-valued arguments (output variables are lists)
    ${ARGN} # arguments of the function to parse, here we take the all original ones
  )

  # Install export targets unless explicitly asked to not do so
  if(NOT ARG_NO_EXPORT_TARGETS)
    wombat_install_export_targets()
  endif()

  # Export package dependencies
  if(ARG_EXPORT_DEPS)
    # Make sure that all the dependencies that we are trying to export are valid.
    # This requires that they have been find_package'd by this package CMakeLists.txt
    foreach(_dep ${ARG_EXPORT_DEPS})
      if(NOT ${_dep}_FOUND)
        message(FATAL_ERROR "Trying to export ${_dep} but it was not found.")
      endif()
    endforeach()

    find_package(ament_cmake_export_dependencies REQUIRED)
    ament_export_dependencies("${ARG_EXPORT_DEPS}")
  endif()

  # Mark this as a package and complete install setup
  ament_package("${ARG_AMENT_PACKAGE}")
endmacro()
