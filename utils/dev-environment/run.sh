#!/bin/bash

# Exit on errors
# https://www.gnu.org/software/bash/manual/html_node/The-Set-Builtin.html
set -o errexit

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
SYMLINKED_DIR="$(dirname "$(readlink -f "$0")")"

DOCKERFILE=${THIS_DIR}/Dockerfile
SETUP_SCRIPT=${SYMLINKED_DIR}/setup-docker.sh
UTILS_SCRIPT=${SYMLINKED_DIR}/utils.sh
EXPECTED_VERSION_FILE=${SYMLINKED_DIR}/VERSION

source ${UTILS_SCRIPT}

if [ ! -f ${DOCKERFILE} ]; then
  echo "### - Error! ${DOCKERFILE} not found!"
  echo "### - Run ${SETUP_SCRIPT} before this script"
  exit 1
fi

if [ ! -f ${EXPECTED_VERSION_FILE} ]; then
  echo "### - Error! ${EXPECTED_VERSION_FILE} not found!"
  echo "### - Run ${SETUP_SCRIPT} before this script"
  exit 1
fi

IS_UBUNTU_HOST=$(is_ubuntu_host)

EXPECTED_VERSION=$(head -n 1 ${EXPECTED_VERSION_FILE})
THIS_FROM_COMMAND=$(head -n 1 ${DOCKERFILE})
EXPECTED_FROM_COMMAND="FROM ${EXPECTED_VERSION}"
if [ "${THIS_FROM_COMMAND}" != "${EXPECTED_FROM_COMMAND}" ]; then
  echo "#########################################################################"
  echo "WARNING! Your Dockerfile is not the expected one for this branch."
  echo "Your base image is ${THIS_FROM_COMMAND}"
  echo "The expected base image is ${EXPECTED_VERSION}"
  echo "Run the setup script to restore the expected base image:"
  echo "bash ${SETUP_SCRIPT}"
  echo "#########################################################################"
fi

# Build the developer's dockerfile
# NOTE: disable buildkit which requires internet connection for building.
DOCKER_BUILDKIT=0 docker build \
  --file ${DOCKERFILE} \
  --network=host \
  --tag wombat-dev \
  ${THIS_DIR}

# Define some useful directory names
WOMBAT_DIR=$(dirname ${THIS_DIR})
DOCKER_HOME=/home/docker-dev
DOCKER_WOMBAT_DIR=${DOCKER_HOME}/wombat

# Persistent bash history file
BASH_HISTORY_FILE=${THIS_DIR}/.bash_history
DOCKER_BASH_HISTORY_FILE=${DOCKER_HOME}/.bash_history
if [ ! -f ${BASH_HISTORY_FILE} ]; then
  touch ${BASH_HISTORY_FILE}
fi
BASH_HISTORY_ARGS="--volume=${BASH_HISTORY_FILE}:${DOCKER_BASH_HISTORY_FILE}:rw"

# Persistent ccache directory
CCACHE_DIR=${THIS_DIR}/.cache/ccache
DOCKER_CCACHE_DIR=${DOCKER_HOME}/.cache/ccache
if [ ! -d $CCACHE_DIR ]; then
  mkdir -p ${CCACHE_DIR}
fi
CCACHE_ENV_ARG="--env CCACHE_DIR=${DOCKER_CCACHE_DIR}"
CCACHE_VOLUME_ARG="--volume=${CCACHE_DIR}:${DOCKER_CCACHE_DIR}:rw"
CCACHE_ARGS="${CCACHE_ENV_ARG} ${CCACHE_VOLUME_ARG}"

# Check if we have gitconfig
GITCONFIG=${HOME}/.gitconfig
GITCONFIG_ARGS=""
if [ -f $GITCONFIG ]; then
  DOCKER_GITCONFIG=${DOCKER_HOME}/.gitconfig
  # Make this volume read-only
  GITCONFIG_ARGS="--volume=${GITCONFIG}:${DOCKER_GITCONFIG}:ro"
fi

# Check if we have display
DISPLAY_ARGS=""
if [ ! -z "${DISPLAY}" ]; then
  XSOCK=/tmp/.X11-unix
  DISPLAY_VOLUMES="--volume=${XSOCK}:${XSOCK}:rw"
  DISPLAY_ENV=""
  if [ -n "${IS_UBUNTU_HOST}" ]; then
    # Make sure XAUTH exists
    XAUTH=/tmp/.docker.xauth
    touch ${XAUTH}
    xauth nlist ${DISPLAY} | sed -e 's/^..../ffff/' | xauth -f ${XAUTH} nmerge -

    DISPLAY_ENV="--env DISPLAY=${DISPLAY} --env XAUTHORITY=${XAUTH}"
    DISPLAY_VOLUMES="${DISPLAY_VOLUMES} --volume=${XAUTH}:${XAUTH}:rw"
  else
    IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
    /usr/X11/bin/xhost + ${IP}
    DISPLAY_ENV="--env DISPLAY=${IP}:0"
  fi

  DISPLAY_ARGS="${DISPLAY_ENV} ${DISPLAY_VOLUMES}"
fi

# Check if we have GPUs
GPU_ARGS=""
if [ -n "${IS_UBUNTU_HOST}" ]; then
  if type "nvidia-container-cli" &> /dev/null && nvidia-container-cli info &> /dev/null; then 
    GPU_ARGS="--gpus=all"
  fi
fi

# Check if we have ssh agent
SSH_ARGS=""
if [ ! -z "${SSH_AUTH_SOCK}" ]; then
  if [ -n "${IS_UBUNTU_HOST}" ]; then
    SSH_AGENT_DIR=$(readlink -f ${SSH_AUTH_SOCK})
    SSH_AGENT_ARGS="--volume=${SSH_AGENT_DIR}:/ssh-agent:ro --env SSH_AUTH_SOCK=/ssh-agent"
    KNOWN_HOSTS="--volume=${HOME}/.ssh/known_hosts:${DOCKER_HOME}/.ssh/known_hosts:ro"
    SSH_ARGS="${SSH_AGENT_ARGS} ${KNOWN_HOSTS}"
  else
    # Mounting the whole .ssh directory is not ideal, but it's the only solution I found that works on MacOS
    SSH_ARGS="--volume=${HOME}/.ssh:${DOCKER_HOME}/.ssh:ro"
  fi
fi

NETWORK_ARGS=""
# Linux allows to use --network=host setting
if [ -n "${IS_UBUNTU_HOST}" ]; then
  NETWORK_ARGS="--network=host"
else
  # 8765 is the port used by the Foxglove bridge
  NETWORK_ARGS="-p 8765:8765"
fi

# Run the developer's dockerfile
docker run -it --rm \
  ${BASH_HISTORY_ARGS} \
  ${CCACHE_ARGS} \
  ${DISPLAY_ARGS} \
  ${GITCONFIG_ARGS} \
  ${GPU_ARGS} \
  ${SSH_ARGS} \
  ${NETWORK_ARGS} \
  --privileged \
  --user=${UID}:${UID} \
  --volume=${HOME}:${DOCKER_HOME}/host-home:rw \
  --volume=${WOMBAT_DIR}:${DOCKER_WOMBAT_DIR}:rw \
  --env COLCON_HOME=${DOCKER_WOMBAT_DIR}/.colcon \
  --workdir ${DOCKER_WOMBAT_DIR} \
  wombat-dev \
  $@
