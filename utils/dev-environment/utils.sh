#!/bin/bash

# Checks if the OS is Ubuntu, returns empty string otherwise
is_ubuntu_host() {
  LINUX_RELEASE_FILE=/etc/os-release
  if [ -f $LINUX_RELEASE_FILE ]; then
    IS_UBUNTU_HOST=$(awk -F= '/^NAME/{print $2}' ${LINUX_RELEASE_FILE} | grep Ubuntu)
  fi
  echo ${IS_UBUNTU_HOST}
}

versioncmp() {
  # From https://stackoverflow.com/a/4024263
  [  "$1" = "`echo -e "$1\n$2" | sort -V | head -n1`" ]
}
