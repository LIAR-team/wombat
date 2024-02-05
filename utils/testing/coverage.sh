#!/bin/bash

# Exit on errors
# https://www.gnu.org/software/bash/manual/html_node/The-Set-Builtin.html
set -o errexit

THIS_DIR=$(dirname $(realpath -s $0))
WOMBAT_DIR=${THIS_DIR}/../..

# Change directory to the root of the repository
cd ${WOMBAT_DIR}

# Build workspace
colcon build --mixin wombat coverage
# Run tests
colcon test --mixin no-lint coverage

WORKSPACE_DIR=${WOMBAT_DIR}/_ws
BUILD_DIR=${WORKSPACE_DIR}/build
INSTALL_DIR=${WORKSPACE_DIR}/install
RELATIVE_INSTALL_DIR=$(realpath  --relative-to="${WOMBAT_DIR}" "${INSTALL_DIR}")

# Exclude packages:
# - *_msgs packages contain automatically generated files
EXCLUDE_PACKAGES=$(
  colcon list \
    --names-only \
    --packages-select-regex \
      ".*_msgs" \
  | xargs)
# Include every other package
INCLUDE_PACKAGES=$(
  colcon list \
    --names-only \
    --packages-ignore \
      $EXCLUDE_PACKAGES \
  | xargs)

COVERAGE_FILE=${WORKSPACE_DIR}/coverage.info
# Remove old coverage file
if [ -f "${COVERAGE_FILE}" ]; then
  rm ${COVERAGE_FILE}
fi

# Capture executed code data.
fastcov --lcov \
  -d ${BUILD_DIR} \
  --exclude test/ ${RELATIVE_INSTALL_DIR} ${EXCLUDE_PACKAGES} \
  --include ${INCLUDE_PACKAGES} \
  --process-gcno \
  --validate-sources \
  --dump-statistic \
  --output ${COVERAGE_FILE}

HTML_DIR=${WORKSPACE_DIR}/coverage
# Remove old html directory
if [ -d "${HTML_DIR}" ]; then
  rm -rf ${HTML_DIR}
fi

# Generate html visualization
genhtml ${COVERAGE_FILE} --output-directory ${HTML_DIR}
