# Copyright 2022 Soragna Alberto.
# All Rights Reserved.
# Unauthorized copying via any medium is strictly prohibited.
# Proprietary and confidential.

#
# Internal utility function used to invoke the `install_hooks.sh` shell script from CMake.
# This will install the githooks into this repository `.git` directory.
#
# @private
#
macro(run_install_githooks_script)

  set(GITHOOKS_CONFIG_FILE ${CMAKE_SOURCE_DIR}/git-hooks-config.yaml)

  # Add a configuration-time dependency on the githook files to re-run the install script when
  # a change is detected (i.e. when a change is detected we force a reconfiguration of this package).
  set_property(
    DIRECTORY
    APPEND
    PROPERTY
      CMAKE_CONFIGURE_DEPENDS
      ${GITHOOKS_CONFIG_FILE}
  )

  # Install repository githooks as part of this package CMake configuration step.
  execute_process(
    COMMAND pre-commit install --overwrite --config ${GITHOOKS_CONFIG_FILE}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE install_hooks_ret
  )
  if(install_hooks_ret)
    message(FATAL_ERROR "Failed to install git hooks: '${install_hooks_ret}'")
  endif()
endmacro()
