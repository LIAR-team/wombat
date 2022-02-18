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

THIS_FROM_COMMAND=$(head -n 1 $DOCKERFILE)
EXPECTED_FROM_COMMAND="FROM $(head -n 1 $EXPECTED_VERSION_FILE)"
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

# Make sure XAUTH exists
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch ${XAUTH}
xauth nlist ${DISPLAY} | sed -e 's/^..../ffff/' | xauth -f ${XAUTH} nmerge -

# Check if we have GPUs
GPU_FLAG=""
if type "nvidia-container-cli" &> /dev/null && nvidia-container-cli info &> /dev/null; then 
  GPU_FLAG="--gpus=all"
fi

# Define some useful directory names
WOMBAT_DIR=$(dirname ${THIS_DIR})
DOCKER_HOME=/home/docker-dev
DOCKER_WOMBAT_WS=${DOCKER_HOME}/wombat

# Run the developer's dockerfile
docker run -it --rm \
  --env="DISPLAY=${DISPLAY}" \
  --env="XAUTHORITY=${XAUTH}" \
  ${GPU_FLAG} \
  --network=host \
  --privileged \
  --user=${UID}:${UID} \
  --volume=${XSOCK}:${XSOCK}:rw \
  --volume=${XAUTH}:${XAUTH}:rw \
  --volume=${HOME}:${DOCKER_HOME}/host-home:rw \
  --volume=${HOME}/.gitconfig:${DOCKER_HOME}/.gitconfig:ro \
  --volume=${HOME}/.ssh:${DOCKER_HOME}/.ssh:ro \
  --volume=${WOMBAT_DIR}:${DOCKER_WOMBAT_WS}:rw \
  --workdir ${DOCKER_WOMBAT_WS} \
  wombat-dev \
  $@
