FROM wombat-base

# Build-time arguments
ARG USERNAME=docker-dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set username and home environment variables
ENV USERNAME $USERNAME
ENV HOME /home/$USERNAME

# Create a new user with the provided details
RUN groupadd --gid $USER_GID $USERNAME && \
  useradd --create-home --home-dir /home/$USERNAME --shell /bin/bash --uid $USER_UID --gid $USERNAME -m $USERNAME

# Add the new user to the sudoers group with no password
RUN echo "$USERNAME:x:$USER_UID:$USER_GID:Developer,,,:$HOME:/bin/bash" >> /etc/passwd && \
  echo "$USERNAME:x:$USER_GID:" >> /etc/group && \
  echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME && \
  chown $USER_UID:$USER_GID -R $HOME

# Start using the new user
USER $USERNAME

# Get ownership of the /workspaces directory
# Not recursive ownership, see this bug (https://github.com/docker/for-linux/issues/388)
RUN sudo chown $USERNAME:$USERNAME /workspaces

# Create wombat workspace where this repo will be mounted as a volume
RUN mkdir -p /workspaces/wombat_ws/src/wombat && \
  sudo chown -R $USERNAME:$USERNAME /workspaces/wombat_ws
  # && \
  #sudo chown -R $USERNAME:$USERNAME /workspaces/wombat_ws/src && \
  #sudo chown -R $USERNAME:$USERNAME /workspaces/wombat_ws/src/wombat

RUN echo ' \n\
  source /workspaces/ros2_galactic/install/setup.sh' >> $HOME/.bashrc
