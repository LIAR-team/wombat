#!/bin/bash

THIS_DIR=$(dirname $(realpath -s $0))

# Build the developer's dockerfile
docker build \
  --file $THIS_DIR/Dockerfile \
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
