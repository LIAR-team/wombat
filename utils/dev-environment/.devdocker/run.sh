#!/bin/bash

THIS_DIR=$(dirname $(realpath -s $0))
WOMBAT_DIR=$(dirname ${THIS_DIR})
CURR_RELATIVE_PATH=$(realpath --relative-to="${WOMBAT_DIR}" "${PWD}")
echo $CURR_RELATIVE_PATH
if [ "${CURR_RELATIVE_PATH##.}" != "${CURR_RELATIVE_PATH}" ]; then
  CURR_RELATIVE_PATH=.
fi

echo $CURR_RELATIVE_PATH

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

# Check if this directory is in a workspace already
# NOTE: this assumes that workspaces follow the `colcon` naming scheme
#if [ $(basename $(dirname ${WOMBAT_DIR})) == "src" ]; then
#  HOST_WOMBAT_WS_DIR=$(dirname $(dirname ${WOMBAT_DIR}))
#else
#  HOST_WOMBAT_WS_DIR=$WOMBAT_DIR
#fi
#DOCKER_WOMBAT_DIR_NAME=$(basename ${HOST_WOMBAT_WS_DIR})
#DOCKER_WOMBAT_DIR="/workspaces/${DOCKER_WOMBAT_DIR_NAME}"

DOCKER_HOME=/home/docker-dev

# Run the developer's dockerfile
docker run -it --rm \
  --env="DISPLAY=${DISPLAY}" \
  --env="XAUTHORITY=${XAUTH}" \
  ${GPU_FLAG} \
  --network=host \
  --privileged \
  --volume=${XSOCK}:${XSOCK}:rw \
  --volume=${XAUTH}:${XAUTH}:rw \
  --volume=${HOME}:${DOCKER_HOME}/host-home:rw \
  --volume=${HOME}/.gitconfig:${DOCKER_HOME}/.gitconfig:ro \
  --volume=${HOME}/.ssh:${DOCKER_HOME}/.ssh:ro \
  #--volume=${HOST_WOMBAT_WS_DIR}:${DOCKER_WOMBAT_DIR} \
  #--workdir ${DOCKER_WOMBAT_DIR}/${CURR_RELATIVE_PATH} \
  wombat-dev \
  $@
