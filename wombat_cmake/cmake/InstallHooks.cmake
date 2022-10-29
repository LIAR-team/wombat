# Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
# All Rights Reserved.

#
# Internal utility function used to invoke the `install_hooks.sh` shell script from CMake.
# This will install the githooks into this repository `.git` directory.
#
# @private
#
macro(run_install_githooks_script)
  # List of githook files.
  # If a file is not here, we will monitor it for changes (added/removed/modified).
  set(GITHOOK_FILES
    ${CMAKE_SOURCE_DIR}/githooks/install_hooks.sh
    ${CMAKE_SOURCE_DIR}/githooks/pre-push
  )

  # Add a configuration-time dependency on the githook files to re-run the install script when
  # a change is detected (i.e. when a change is detected we force a reconfiguration of this package).
  set_property(
    DIRECTORY
    APPEND
    PROPERTY
      CMAKE_CONFIGURE_DEPENDS
      ${GITHOOK_FILES}
  )

  # Install repository githooks as part of this package CMake configuration step.
  execute_process(
    COMMAND ${CMAKE_SOURCE_DIR}/githooks/install_hooks.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE install_hooks_ret
  )
  if(install_hooks_ret)
    message(FATAL_ERROR "Failed to install git hooks: '${install_hooks_ret}'")
  endif()
endmacro()
