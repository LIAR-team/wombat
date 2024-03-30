# Copyright 2021-2022 Soragna Alberto.

# For more information see:
# https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Debugging-Options.html#Debugging-Options
# https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Optimize-Options.html#Optimize-Options

# Release
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG"
  CACHE STRING "Flags used by the C++ compiler for builds optimized for speed." FORCE
)
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG"
  CACHE STRING "Flags used by the C compiler for builds optimized for speed." FORCE
)

# RelWithDebug
set(CMAKE_CXX_FLAGS_RELWITHDEBUG "-O2 -g -DDEBUG"
  CACHE STRING "Flags used by the C++ compiler for fast builds with full debug support." FORCE
)
set(CMAKE_C_FLAGS_RELWITHDEBUG "-O2 -g -DDEBUG"
  CACHE STRING "Flags used by the C compiler for fast builds with full debug support." FORCE
)

set(_allowedBuildTypes "RELEASE;RELWITHDEBUG")

#
# Makes sure that the CMAKE_BUILD_TYPE variable is properly set.
#
# To use this macro it is necessary to:
# - add `<buildtool_depend>wombat_cmake</buildtool_depend>` to your project package.xml file
# - add `find_package(wombat_cmake REQUIRED)` to your project main CMakeLists.txt
#
# @public
#
macro(handle_cmake_build_type)
  if(NOT CMAKE_BUILD_TYPE)
    # Set default build type
    message(STATUS "Setting build type to RelWithDebug as none was specified.")
    set(CMAKE_BUILD_TYPE RELWITHDEBUG CACHE STRING "" FORCE)
  elseif(NOT ${CMAKE_BUILD_TYPE} IN_LIST _allowedBuildTypes)
    # The provided type is not valid
    message(FATAL_ERROR "Invalid build type: ${CMAKE_BUILD_TYPE}. Please select one of ${_allowedBuildTypes}")
  endif()
endmacro()
