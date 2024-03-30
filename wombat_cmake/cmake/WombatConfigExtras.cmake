# Copyright 2024 Soragna Alberto.

include(CMakeParseArguments)

#
# Write a CMake config extras file to install cmake files that can be consumed
# by clients of this package.
# See CONFIG_EXTRAS details at
# https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html#basic-project-outline
#
# @public
#
function(wombat_write_config_extras)
  cmake_parse_arguments(
    ARG # prefix of output variables
    "" # list of names of the boolean arguments (only defined ones will be true)
    "CMAKE_DIR_VAR_NAME;OUTPUT_PATH" # list of names of mono-valued arguments
    "SCRIPT_NAMES" # list of names of multi-valued arguments (output variables are lists)
    ${ARGN} # arguments of the function to parse, here we take the all original ones
  )
  if(NOT ARG_CMAKE_DIR_VAR_NAME)
    set(ARG_CMAKE_DIR_VAR_NAME "${PROJECT_NAME}_DIR")
  endif()
  if(NOT ARG_OUTPUT_PATH)
    message(FATAL_ERROR "Called wombat_write_config_extras without OUTPUT_PATH")
    return()
  endif()
  if(NOT ARG_SCRIPT_NAMES)
    message(FATAL_ERROR "Called wombat_write_config_extras without SCRIPT_NAMES")
    return()
  endif()

  if(NOT ${ARG_CMAKE_DIR_VAR_NAME})
    message(FATAL_ERROR "Variable named '${ARG_CMAKE_DIR_VAR_NAME}' must be set before calling wombat_write_config_extras")
  endif()
  set(SCRIPTS_PREFIX "${CMAKE_SOURCE_DIR}/${${ARG_CMAKE_DIR_VAR_NAME}}")
  if(NOT EXISTS ${SCRIPTS_PREFIX} OR NOT IS_DIRECTORY ${SCRIPTS_PREFIX})
    message(FATAL_ERROR "wombat_write_config_extras got invalid directory ${SCRIPTS_PREFIX}")
    return()
  endif()

  # Write the CMAKE_SCRIPTS_INCLUDE variable
  set(CMAKE_SCRIPTS_INCLUDE "")
  foreach(NAME ${ARG_SCRIPT_NAMES})
    set(ABS_NAME "${SCRIPTS_PREFIX}/${NAME}")
    if(NOT EXISTS "${ABS_NAME}")
      message(FATAL_ERROR "File not found: '${ABS_NAME}'. Check that the file name and the prefix are both correct.")
      return()
    endif()
    # This produces a string like
    # include("${wombat_cmake_DIR}/NAME_VALUE")
    string(APPEND CMAKE_SCRIPTS_INCLUDE "include(\"\${${ARG_CMAKE_DIR_VAR_NAME}}/${NAME}\")\n")
  endforeach()

  # Configure the file
  set(INPUT_FILE ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/AmentConfigExtras.cmake.in)
  configure_file(${INPUT_FILE} ${ARG_OUTPUT_PATH} @ONLY)
endfunction()
