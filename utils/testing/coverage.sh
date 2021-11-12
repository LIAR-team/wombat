#!/bin/bash

THIS_DIR=$(dirname $(realpath -s $0))
WOMBAT_DIR=$THIS_DIR/../..

# Change directory to the root of the repository
cd $WOMBAT_DIR

# Build workspace
colcon build --mixin wombat coverage
# Run tests
colcon test --mixin no-linters coverage

# Include all packages
INCLUDE_PACKAGES=$(
  colcon list \
    --names-only \
  | xargs)

WORKSPACE_DIR=$WOMBAT_DIR/_ws
BUILD_DIR=$WORKSPACE_DIR/build
COVERAGE_FILE=$WORKSPACE_DIR/coverage.info

# Capture executed code data.
fastcov --lcov \
  -d ${BUILD_DIR} \
  --exclude test/ \
  --include ${INCLUDE_PACKAGES} \
  --process-gcno \
  --validate-sources \
  --dump-statistic \
  --output ${COVERAGE_FILE}

HTML_DIR=$WORKSPACE_DIR/coverage

# Generage html visualization
genhtml ${COVERAGE_FILE} --output-directory ${HTML_DIR}
