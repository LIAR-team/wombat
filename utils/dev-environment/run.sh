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

# Check if the developer dockerfile is using the expected version
# of the base image. Print a big warning if it is not.
EXPECTED_VERSION=$(head -n 1 ${EXPECTED_VERSION_FILE})
THIS_FROM_COMMAND=$(head -n 1 ${DOCKERFILE})
THIS_VERSION=${THIS_FROM_COMMAND#"FROM "}
if [ "${THIS_VERSION}" != "${EXPECTED_VERSION}" ]; then
  echo "#########################################################################"
  echo "WARNING! Your Dockerfile is not the expected one for this branch."
  echo "Your base image is ${THIS_VERSION}"
  echo "The expected base image is ${EXPECTED_VERSION}"
  echo "Run the setup script to restore the expected base image:"
  echo "bash ${SETUP_SCRIPT}"
  echo "#########################################################################"
fi

# Build the developer's dockerfile
DOCKER_CLI_HINTS=false docker build \
  --file ${DOCKERFILE} \
  --network=host \
  --tag wombat-dev \
  ${THIS_DIR}

# Define some useful directory names
WOMBAT_DIR=$(dirname ${THIS_DIR})
DOCKER_HOME=/home/docker-dev
DOCKER_WOMBAT_DIR=${DOCKER_HOME}/wombat

# Make sure that the mounted directory is in the "safe" list of git
GIT_CONFIG_SAFE_DIRS_KEY="safe.directory"
GIT_CONFIG_SAFE_DIRS_VALUES=$( git config --global --get "${GIT_CONFIG_SAFE_DIRS_KEY}" || true; )
if [[ "${GIT_CONFIG_SAFE_DIRS_VALUES}" != *"${DOCKER_WOMBAT_DIR}"* ]]; then
  echo "The git config '${GIT_CONFIG_SAFE_DIRS_KEY}' does not contain the mounted workspace directory '${DOCKER_WOMBAT_DIR}'. Adding it."
  git config --global --add ${GIT_CONFIG_SAFE_DIRS_KEY} ${DOCKER_WOMBAT_DIR}
fi

# Persistent bash history file
BASH_HISTORY_FILE=${THIS_DIR}/.bash_history
DOCKER_BASH_HISTORY_FILE=${DOCKER_HOME}/.bash_history
if [ ! -f ${BASH_HISTORY_FILE} ]; then
  touch ${BASH_HISTORY_FILE}
fi
BASH_HISTORY_ARGS="--volume=${BASH_HISTORY_FILE}:${DOCKER_BASH_HISTORY_FILE}:rw"

# Utility to create a local cache directory, mount it as a volume and point an env variable to it
setup_cache_dir() {
  DIR_NAME=$1
  ENV_VAR_NAME=$2

  if [ -z "${DIR_NAME}" ]; then
    echo "No directory name passed to setup_cache_dir" >&2
    exit 1
  fi
  LOCAL_DIR=${THIS_DIR}/.cache/${DIR_NAME}
  DOCKER_DIR=${DOCKER_HOME}/.cache/${DIR_NAME}
  if [ ! -d "${LOCAL_DIR}" ]; then
    mkdir -p ${LOCAL_DIR}
  fi
  VOLUME_ARG="--volume=${LOCAL_DIR}:${DOCKER_DIR}:rw"
  ENV_ARG=""
  if [ ! -z "${ENV_VAR_NAME}" ]; then
    ENV_ARG="--env ${ENV_VAR_NAME}=${DOCKER_DIR}"
  fi

  echo "${VOLUME_ARG} ${ENV_ARG}"
}

CACHE_ARGS="$(setup_cache_dir ccache CCACHE_DIR) $(setup_cache_dir pre-commit PRE_COMMIT_HOME)"

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
    # TODO: this may not work on every PC, i.e. the interface may be named differently
    NETWORK_INTERFACE="en0"
    IP=$(ifconfig ${NETWORK_INTERFACE} | grep inet | awk '$1=="inet" {print $2}')
    # If IP is not found, we don't want to run this command.
    # It would be a safety issue as it would allow any IP to connect.
    if [[ -n "${IP}" ]]; then
      /usr/X11/bin/xhost + ${IP}
      DISPLAY_ENV="--env DISPLAY=${IP}:0"
    fi
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

# Define the full command to start the container
CMD=(docker run -it --rm \
  ${BASH_HISTORY_ARGS} \
  ${CACHE_ARGS} \
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
  $@)

# Print the command we are about to execute
echo "####
${CMD[@]}
####"
# Run the command to start a container for the developer's dockerfile
"${CMD[@]}"
