#!/bin/bash

# This script will run doxygen on the whole codebase.
# The output will contain pdf/html files used to visualize the documentation
# This will be written in the OUTPUT_DIRECTORY indicated below.
# To visualize the html documentation, just open the index.html file in a
# web browser.

# Exit on errors
# https://www.gnu.org/software/bash/manual/html_node/The-Set-Builtin.html
set -o errexit

THIS_DIR=$(dirname $(realpath -s $0))
CONFIG_FILE=${THIS_DIR}/config
WORKSPACE_DIR=${THIS_DIR}/../../_ws
OUTPUT_DIRECTORY=${WORKSPACE_DIR}/doxygen

if [ ! -f ${CONFIG_FILE} ]; then
  echo "Doxygen config file not found: ${CONFIG_FILE}"
  exit 1
fi 

# Override the doxygen `OUTPUT_DIRECTORY` field to point to the workspace
# and ensure that it exists
if [ ! -d ${WORKSPACE_DIR} ]; then
  mkdir ${WORKSPACE_DIR}
fi

# See FAQs on how to override configuration https://www.doxygen.nl/manual/faq.html#faq_cmdline
( cat ${CONFIG_FILE} ; echo "OUTPUT_DIRECTORY=${OUTPUT_DIRECTORY}" ) | doxygen -
