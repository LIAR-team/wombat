#!/bin/bash

# Checks if the OS is Ubuntu, returns empty string otherwise
is_ubuntu_host() {
  LINUX_RELEASE_FILE=/etc/os-release
  if [ -f $LINUX_RELEASE_FILE ]; then
    IS_UBUNTU_HOST=$(awk -F= '/^NAME/{print $2}' ${LINUX_RELEASE_FILE} | grep Ubuntu)
  fi
  echo ${IS_UBUNTU_HOST}
}

# Checks if the first version argument is smaller or equal than the second one
# NOTE: this utility is Linux-only!
# From https://stackoverflow.com/a/4024263
versioncmp() {
  [  "$1" = "`echo -e "$1\n$2" | sort -V | head -n1`" ]
}

# Create a symlink to the first path at the destination path.
# If ANYTHING already exists at the destination path, it will be destroyed.
create_symlink() {
  ORIGINAL_PATH=$1
  SYMLINKED_PATH=$2

  if [ ! -e ${ORIGINAL_FILE} ]; then
    echo "The path to symlink does not exist: ${ORIGINAL_PATH}"
    exit 1
  fi

  # If the symlinked path already exists
  if [ -e ${SYMLINKED_PATH} ]; then
    # If it's not a link or if it does not point to the expected path, remove it
    if [ ! -L ${SYMLINKED_PATH} ] || [ ! "${SYMLINKED_PATH}" -ef "${ORIGINAL_PATH}" ]; then
      rm -rf ${SYMLINKED_PATH}
    fi
  fi

  # If the symlink path does not exist, create it
  if [ ! -L ${SYMLINKED_PATH} ]; then
    ln -s ${ORIGINAL_PATH} ${SYMLINKED_PATH}
  fi
}
