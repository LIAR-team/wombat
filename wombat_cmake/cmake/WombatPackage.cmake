# Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
# All Rights Reserved.

#
# Standard Wombat project setup.
# Call `wombat_package()` at the beginning of your Wombat CMake project in order
# to setup all common compilation flags and options.
#
# Note that, in order to use this macro, it is necessary to:
# - add `<build_depend>wombat_cmake</build_depend>` to your project package.xml file
# - add `find_package(wombat_cmake REQUIRED)` to your project main CMakeLists.txt
#
# @public
#
macro(wombat_package)

  # Handle CMake build types
  include(${wombat_cmake_DIR}/BuildTypes.cmake)
  set(_allowed_build_types RELEASE RELWITHDEBUG)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "${_allowed_build_types}")
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to RelWithDebug as none was specified.")
    set(CMAKE_BUILD_TYPE RELWITHDEBUG CACHE STRING "" FORCE)
  elseif(NOT ${CMAKE_BUILD_TYPE} IN_LIST _allowed_build_types)
    # Verify the correct build type
    message(FATAL_ERROR "Invalid build type: ${CMAKE_BUILD_TYPE}. Please select one of ${_allowed_build_types}")
  endif()

  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()

  # Set default flags
  # See https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Warning-Options.html#Warning-Options
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
      -Werror
      -Wall
      -Wextra
      -Wpedantic
      -Wdeprecated
      -Wimplicit-fallthrough
      -Wmissing-braces
      -Wmissing-declarations
      -Wnull-dereference
      -Wredundant-decls
      -Wshadow
      -fPIC
    )
  endif()

  option(COVERAGE_ENABLED "Enable code coverage" FALSE)
  if(COVERAGE_ENABLED)
    add_compile_options(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
  endif()

endmacro()
