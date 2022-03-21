# Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
# All Rights Reserved.

#
# Installs a target and its include directories.
#
# Note that, in order to use this function, it is necessary to:
# - add `<build_depend>wombat_cmake</build_depend>` to your project package.xml file
# - add `find_package(wombat_cmake REQUIRED)` to your project main CMakeLists.txt
#
# @public
#
function(wombat_install_target TARGET_NAME)

  if(NOT TARGET ${TARGET_NAME})
    message(FATAL_ERROR "Target ${TARGET_NAME} passed to wombat_install_target does not exist")
  endif()

  install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${TARGET_NAME}/include
  )

  install(
    TARGETS ${TARGET_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION ${TARGET_NAME}/include
  )

endfunction()
