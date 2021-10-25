#!/bin/bash

THIS_DIR=$(dirname $(realpath -s $0))
REPO_DIR=$THIS_DIR/../..

# Change directory to the root of the repository
cd $REPO_DIR

# Flag to notify user if this script installed something that requires restart
RESTART_SUGGESTED=0

# Make sure docker is already installed on the system
if [ -x "$(command -v docker)" ]; then
  echo "### - docker is already installed on the system. Skipping ..."
elif [ -n "$(awk -F= '/^NAME/{print $2}' /etc/os-release | grep Ubuntu)" ]; then
  echo "Installing docker"
  # Docs https://docs.docker.com/engine/install/ubuntu/
  sudo apt-get remove docker docker-engine docker.io containerd runc
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
  sudo apt-key fingerprint 0EBFCD88
  sudo add-apt-repository \
    "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) \
    stable"
  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io
  RESTART_SUGGESTED=1
else
  echo "You are not running an Ubuntu system. Install Docker yourself and come back here!"
  exit 1
fi

if lshw -C display | grep -q 'NVIDIA'; then
  echo "### - found NVIDIA GPU in the system"
  # Docs https://stackoverflow.com/a/58432877/7108533
  # This assumes that NVIDIA drivers have been setup by the user
  if [ -x "$(command -v nvidia-container-toolkit)" ]; then
    echo "### - nvidia-container-toolkit is already installed on the system. Skipping ..."
  elif [ -n "$(awk -F= '/^NAME/{print $2}' /etc/os-release | grep Ubuntu)" ]; then
    echo "Installing nvidia-container-toolkit"
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    sudo apt-get update && sudo apt-get install -y --no-install-recommends curl
    curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
    sudo apt-get update && sudo apt-get install -y --no-install-recommends nvidia-container-toolkit
    sudo systemctl restart docker
    RESTART_SUGGESTED=1
  else
    echo "You are not running an Ubuntu system. Install Docker yourself and come back here!"
    exit 1
  fi
fi

# Build base docker image
docker build \
  --build-arg USERNAME="docker-dev" \
  --build-arg USER_UID="$UID" \
  --file $THIS_DIR/Dockerfile \
  --network=host \
  --tag wombat-user \
  $THIS_DIR

# Install dev docker scripts
if [ ! -e .devdocker ]; then
  echo "Symlinking utils/dev-environment/.devdocker to .devdocker"
  if [ -L ".devdocker" ]; then
    rm .devdocker
  fi
  ln -s -r utils/dev-environment/.devdocker/ .devdocker
else
  echo "### - .devdocker already exists. Skipping ..."
fi

# Check if a restart is required before starting to work
if [ ! "$RESTART_SUGGESTED" = "0" ]; then
  echo "! ** Some system settings changed, RESTART Suggested ** !"
  read -r -p "Would you like to restart now? [Y/n] " input
  if [[ ! "$input" ]]; then
    input=y
  fi
  case $input in
    [yY][eE][sS]|[yY])
  echo "Restarting..."
  reboot
  ;;
    *)
  echo "Not restarting, you may need to restart before docker containers work"
  exit 1
  ;;
  esac
fi
