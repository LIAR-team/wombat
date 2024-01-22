#!/bin/bash

set -e

THIS_DIR=$(dirname $(realpath -s $0))

# Directory where we need to install git hooks (e.g. .git/hooks)
GIT_HOOKS_DIR="$(realpath $(git rev-parse --git-path hooks))"
# Make sure hooks directory exists
mkdir -p ${GIT_HOOKS_DIR}

# Symlink each file (except this shell script) into .git/hooks
THIS_FILE=$(basename -- "$0")
for file in $(ls -A ${THIS_DIR}); do
  file_abs_path=${THIS_DIR}/${file}
  if [[ "${file}" == "${THIS_FILE}" ]]; then
    echo "Skipping ${file}"
    continue
  elif [[ -x ${file_abs_path} ]] && [[ ! -d ${file_abs_path} ]]; then
    # Symlink all the executable files found in the githooks directory.
    ln -rsf ${file_abs_path} ${GIT_HOOKS_DIR}
    echo "Installed ${file_abs_path} githook"
  else
    echo "Skipping ${file_abs_path} because it is not an executable file"
  fi
done
