# Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
# All Rights Reserved.

include(CMakeParseArguments)

#
# Standard Wombat project setup.
# Call `wombat_package()` at the beginning of your Wombat CMake project in order
# to setup all common compilation flags and options.
#
# To use this macro it is necessary to:
# - add `<buildtool_depend>wombat_cmake</buildtool_depend>` to your project package.xml file
# - add `find_package(wombat_cmake REQUIRED)` to your project main CMakeLists.txt
#
# @public
#
macro(wombat_package)
  cmake_parse_arguments(
    ARG # prefix of output variables
    "NO_WOMBAT_LINTERS" # list of names of the boolean arguments (only defined ones will be true)
    "" # list of names of mono-valued arguments
    "" # list of names of multi-valued arguments (output variables are lists)
    ${ARGN} # arguments of the function to parse, here we take the all original ones
  )

  include(${wombat_cmake_DIR}/BuildTypes.cmake)

  option(WOMBAT_COVERAGE "Enable code coverage" FALSE)

  handle_cmake_build_type()

  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()

  # Set default flags
  # See https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Warning-Options.html#Warning-Options
  # Can't enable -Wredundant-decls due to https://github.com/ros2/rosidl_typesupport_fastrtps/issues/28
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
      -fPIC
      -Wall
      -Wdeprecated
      -Werror
      -Wextra
      -Wimplicit-fallthrough
      -Wmissing-braces
      -Wmissing-declarations
      -Wnull-dereference
      -Wpedantic
      -Wpointer-arith
      -Wshadow
    )
  endif()

  if(WOMBAT_COVERAGE)
    add_compile_options(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
  endif()

  # Automatically run linters
  if(BUILD_TESTING AND NOT ARG_NO_WOMBAT_LINTERS)
    wombat_linters()
  endif()

endmacro()
