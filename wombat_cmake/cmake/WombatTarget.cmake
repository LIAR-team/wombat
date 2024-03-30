# Copyright 2024 Soragna Alberto.

#
# Add a CMake Library target and creates an alias
# namespaced in the current project.
#
# For example, calling `wombat_add_library(my_lib STATIC)` in `MyProject`
# will result in creating the library target `my_lib` and its alias `MyProject::my_lib`.
#
# @public
#
function(wombat_add_library LIBRARY_NAME LIBRARY_TYPE)
  if(NOT LIBRARY_NAME)
    message(FATAL_ERROR "Argument LIBRARY_NAME is null")
  endif()
  if(NOT LIBRARY_TYPE)
    message(FATAL_ERROR "Argument LIBRARY_TYPE is null")
  endif()

  add_library(${LIBRARY_NAME} ${LIBRARY_TYPE})
  add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})
endfunction()
