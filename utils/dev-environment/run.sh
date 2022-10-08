#!/bin/bash

THIS_DIR=$(dirname $(realpath -s $0))
SYMLINKED_DIR="$(dirname "$(readlink -f "$0")")"

DOCKERFILE=$THIS_DIR/Dockerfile
SETUP_SCRIPT=$SYMLINKED_DIR/setup-docker.sh
EXPECTED_VERSION_FILE=$SYMLINKED_DIR/VERSION

if [ ! -f $DOCKERFILE ]; then
  echo "### - Error! $DOCKERFILE not found!"
  echo "### - Run $SETUP_SCRIPT before this script"
  exit 1
fi

if [ ! -f $EXPECTED_VERSION_FILE ]; then
  echo "### - Error! $EXPECTED_VERSION_FILE not found!"
  echo "### - Run $SETUP_SCRIPT before this script"
  exit 1
fi

EXPECTED_VERSION=$(head -n 1 $EXPECTED_VERSION_FILE)
THIS_FROM_COMMAND=$(head -n 1 $DOCKERFILE)
EXPECTED_FROM_COMMAND="FROM $EXPECTED_VERSION"
if [ "$THIS_FROM_COMMAND" != "$EXPECTED_FROM_COMMAND" ]; then
  echo "### - Warning! Your Dockerfile is not up-to-date. The build may fail."
  echo "### - Run $SETUP_SCRIPT to update."
fi

# Build the developer's dockerfile
docker build \
  --file $DOCKERFILE \
  --network=host \
  --tag wombat-dev \
  $THIS_DIR

# Define some useful directory names
WOMBAT_DIR=$(dirname ${THIS_DIR})
DOCKER_HOME=/home/docker-dev
DOCKER_WOMBAT_WS=${DOCKER_HOME}/wombat

# Check if we have display
DISPLAY_ARGS=""
if [ -v DISPLAY ]; then
  # Make sure XAUTH exists
  XSOCK=/tmp/.X11-unix
  XAUTH=/tmp/.docker.xauth
  touch ${XAUTH}
  xauth nlist ${DISPLAY} | sed -e 's/^..../ffff/' | xauth -f ${XAUTH} nmerge -

  DISPLAY_ENV="--env DISPLAY=${DISPLAY} --env XAUTHORITY=${XAUTH}"
  DISPLAY_VOLUMES="--volume=${XSOCK}:${XSOCK}:rw --volume=${XAUTH}:${XAUTH}:rw"

  DISPLAY_ARGS="${DISPLAY_ENV} ${DISPLAY_VOLUMES}"
fi

# Check if we have GPUs
GPU_ARGS=""
if type "nvidia-container-cli" &> /dev/null && nvidia-container-cli info &> /dev/null; then 
  GPU_ARGS="--gpus=all"
fi

SSH_ARGS=""
if [ -v SSH_AUTH_SOCK ]; then
  SSH_AGENT_DIR=$(readlink -f ${SSH_AUTH_SOCK})

  SSH_AGENT_ARGS="--volume=${SSH_AGENT_DIR}:/ssh-agent:ro --env SSH_AUTH_SOCK=/ssh-agent"
  KNOWN_HOSTS="--volume=${HOME}/.ssh/known_hosts:${DOCKER_HOME}/.ssh/known_hosts:ro"
  SSH_ARGS="${SSH_AGENT_ARGS} ${KNOWN_HOSTS}"
fi

# Run the developer's dockerfile
docker run -it --rm \
  ${DISPLAY_ARGS} \
  ${GPU_ARGS} \
  ${SSH_ARGS} \
  --network=host \
  --privileged \
  --user=${UID}:${UID} \
  --volume=${HOME}:${DOCKER_HOME}/host-home:rw \
  --volume=${HOME}/.gitconfig:${DOCKER_HOME}/.gitconfig:ro \
  --volume=${WOMBAT_DIR}:${DOCKER_WOMBAT_WS}:rw \
  --workdir ${DOCKER_WOMBAT_WS} \
  wombat-dev \
  $@
