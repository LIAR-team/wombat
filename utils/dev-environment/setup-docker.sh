#!/bin/bash

# Exit on errors
# https://www.gnu.org/software/bash/manual/html_node/The-Set-Builtin.html
set -o errexit

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
REPO_DIR=${THIS_DIR}/../..
UTILS_SCRIPT=${THIS_DIR}/utils.sh

source ${UTILS_SCRIPT}

# Change directory to the root of the repository
cd ${REPO_DIR}

# Flag to notify user if this script installed something that requires restart
RESTART_SUGGESTED=0

IS_UBUNTU_HOST=$(is_ubuntu_host)

# Make sure docker is already installed on the system
if [ -x "$(command -v docker)" ]; then
  echo "### - docker is already installed on the system."
elif [ -n "${IS_UBUNTU_HOST}" ]; then
  echo "### - Installing docker"
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
  echo "### - You are not running an Ubuntu system. Install Docker yourself and come back here!"
  exit 1
fi

if id -nG "${USER}" | grep -qw "docker"; then
  echo "### - ${USER} belongs to docker group"
elif [ -n "${IS_UBUNTU_HOST}" ]; then
  echo "### - Adding ${USER} to docker group"
  sudo groupadd docker
  sudo usermod -aG docker ${USER}
  RESTART_SUGGESTED=1
fi

# Try to install nvidia GPU drivers and related docker support
if [ -n "${IS_UBUNTU_HOST}" ]; then
  if [ ! -x "$(command -v lshw)" ]; then
    sudo apt-get update && sudo apt-get install -y lshw
  fi
  # NOTE: this command may return error codes (especially if not run as sudo), but it's safe to ignore them
  set +o errexit
  HAS_NVIDIA_GPU=$(lshw -C display && : | grep -q 'NVIDIA')
  set -o errexit
  if [ -n "${HAS_NVIDIA_GPU}" ]; then
    echo "### - Found nvidia GPU in the system"
    # Docs https://stackoverflow.com/a/58432877/7108533
    # This assumes that NVIDIA drivers have been setup by the user
    if [ -x "$(command -v nvidia-container-toolkit)" ]; then
      echo "### - nvidia-container-toolkit is already installed on the system."
    else
      echo "### - Installing nvidia-container-toolkit"
      DISTRIBUTION=$(. /etc/os-release;echo $ID$VERSION_ID)
      sudo apt-get update && sudo apt-get install -y --no-install-recommends curl
      curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
      curl -s -L https://nvidia.github.io/nvidia-docker/${DISTRIBUTION}/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
      sudo apt-get update && sudo apt-get install -y --no-install-recommends nvidia-container-toolkit
      sudo systemctl restart docker
      RESTART_SUGGESTED=1
    fi

    NVIDIA_DRIVER_VERSION=$(nvidia-container-cli info | grep 'NVRM version' | awk '{ print $3 }')
    echo "### - Checking nvidia driver"
    # https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#platform-requirements
    if ( versioncmp "418" $NVIDIA_DRIVER_VERSION ) ; then
      echo "### - nvidia driver version seems ok"
    else
      if (. /etc/os-release; versioncmp "18.04" "$VERSION_ID"); then
        echo "### - Updating nvidia driver version"
        sudo add-apt-repository ppa:graphics-drivers/ppa
        sudo apt-get update
        sudo ubuntu-drivers autoinstall
        echo "### - Driver version updated"
        RESTART_SUGGESTED=1
      else
        echo "### - ERROR: Cannot automatically install nvidia drivers on ubuntu systems older than 18.04"
        exit 1
      fi
    fi
  fi
fi

# Pull base docker image
VERSION_FILE=${THIS_DIR}/VERSION
DEVELOPER_IMAGE=$(head -n 1 ${VERSION_FILE})
docker pull ${DEVELOPER_IMAGE}

# Create user devdocker directory
DEVDOCKER_DIR_NAME=".devdocker"
USER_DEVDOCKER=${REPO_DIR}/${DEVDOCKER_DIR_NAME}
echo "### - Installing developer docker directory to ${USER_DEVDOCKER}"
if [ -e ${USER_DEVDOCKER} ] && [ ! -d ${USER_DEVDOCKER} ]; then
  rm -rf ${USER_DEVDOCKER}
fi
if [ -L ${USER_DEVDOCKER} ]; then
  rm -rf ${USER_DEVDOCKER}
fi
if [ ! -d ${USER_DEVDOCKER} ]; then
  mkdir ${USER_DEVDOCKER}
fi

# Symbolic link files to the user devdocker directory
for filename in "run.sh" ".dockerignore"; do
  ORIGINAL_PATH=${THIS_DIR}/${filename}
  SYMLINKED_PATH=${USER_DEVDOCKER}/${filename}
  create_symlink ${ORIGINAL_PATH} ${SYMLINKED_PATH}
done

# Create the user dockerfile
USER_DOCKERFILE=${USER_DEVDOCKER}/Dockerfile
FROM_IMAGE_CMD="FROM ${DEVELOPER_IMAGE}"
if [ -f ${USER_DOCKERFILE} ]; then
  OLD_FROM_IMAGE_CMD=$(head -n 1 ${USER_DOCKERFILE})
  if [ "${OLD_FROM_IMAGE_CMD}" != "${FROM_IMAGE_CMD}" ]; then
    echo "### - Updating ${USER_DOCKERFILE}"
    echo "### - Replacing \"${OLD_FROM_IMAGE_CMD}\" with \"${FROM_IMAGE_CMD}\""
    # Replace occurrences (all of them!) of OLD_FROM_IMAGE_CMD with FROM_IMAGE_CMD
    # NOTE: MacOS has a different version of sed which requires to specify a backup file
    # NOTE: Use double quotes " " to allow bash variable expansion
    # NOTE: Use '!' as delimiter because strings contain '/' (anything after 's' is the delimiter)
    sed -i.tmp -e "s!${OLD_FROM_IMAGE_CMD}!${FROM_IMAGE_CMD}!" ${USER_DOCKERFILE}
    # Remove the backup file
    rm ${USER_DOCKERFILE}.tmp
  fi
else
  echo "${FROM_IMAGE_CMD}" > ${USER_DOCKERFILE}
  echo "# This is your personal Dockerfile! Add below this line all the tools you want to use." >> ${USER_DOCKERFILE}
fi

# Check if a restart is required before starting to work
if [ ! "${RESTART_SUGGESTED}" = "0" ]; then
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
