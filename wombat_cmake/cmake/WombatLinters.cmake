# Copyright 2021-2022 Soragna Alberto.
# All Rights Reserved.
# Unauthorized copying via any medium is strictly prohibited.
# Proprietary and confidential.

#
# Setup and execute a set of linters depending on the files
# found in the package.
#
# To use this macro it is necessary to:
# - add `<test_depend>wombat_cmake</test_depend>` to your project package.xml file
# - add `find_package(wombat_cmake REQUIRED)` to your project main CMakeLists.txt
#
# @public
#
function(wombat_linters)
  set(_ARGN "${ARGN}")
  if(_ARGN)
    message(FATAL_ERROR "wombat_linters() called with unused arguments: ${_ARGN}")
  endif()

  set(_wombat_linters_dir ${wombat_cmake_DIR}/../linters-config)

  # Search for C/C++ files in this package
  file(GLOB_RECURSE _cpp_files FOLLOW_SYMLINKS
    "*.c"
    "*.cc"
    "*.cpp"
    "*.cxx"
    "*.h"
    "*.hh"
    "*.hpp"
    "*.hxx"
  )

  # Search for Python files in this package
  file(GLOB_RECURSE _python_files FOLLOW_SYMLINKS "*.py")

  # Linters for C/C++ files
  if(_cpp_files)
    # Clang-tidy
    # C/C++ static analysis for various checks
    
    # TODO: clang-tidy is currently broken. Enable it again
    # when https://github.com/ament/ament_lint/pull/441 is merged and released.
    #find_package(ament_cmake_clang_tidy REQUIRED)
    #set(_clang_tidy_config ${_wombat_linters_dir}/clang-tidy-checks)
    ## Include only errors from header files located in the project directory
    #set(_clang_tidy_header_filter --header-filter ${PROJECT_SOURCE_DIR}/.*)
    ## Clang-tidy can be very slow
    #set(_clang_tidy_timeout 1000)
    #ament_clang_tidy(
    #  ${CMAKE_BINARY_DIR}
    #  ${_clang_tidy_header_filter}
    #  CONFIG_FILE ${_clang_tidy_config}
    #  TIMEOUT ${_clang_tidy_timeout}
    #)

    # CppCheck
    # C/C++ static analysis for undefined behaviors and other bugs
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck()

    # Uncrustify
    # Check style for C/C++ files
    find_package(ament_cmake_uncrustify REQUIRED)
    set(_uncrustify_config ${_wombat_linters_dir}/uncrustify-style.cfg)
    ament_uncrustify(CONFIG_FILE ${_uncrustify_config})

    # CPPlint
    # Check style for C/C++ files
    find_package(ament_cmake_cpplint REQUIRED)
    set(_cpplint_filters -readability/todo)
    ament_cpplint(MAX_LINE_LENGTH 120 FILTERS ${_cpplint_filters})
  endif()

  # Linters for Python files
  if(_python_files)
    # Flake8
    # Check style and correctness of Python files
    find_package(ament_cmake_flake8 REQUIRED)
    ament_flake8()
  endif()

  # LintCmake
  # Check style for CMake files
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  # Copyright linter
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

endfunction()
