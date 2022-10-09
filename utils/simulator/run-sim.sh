#!/bin/bash

THIS_DIR=$(dirname $(realpath -s $0))
REPO_DIR=$THIS_DIR/../..

if [ ! -x "$(command -v tmux)" ]; then
  echo "### Installing tmux requirement!"
  sudo apt-get install tmux
fi

TMUX_CONF=$THIS_DIR/tmux.conf
echo $TMUX_CONF

# Create a new tmux session
tmux -f $TMUX_CONF new-session -s main -d bash
# Split the termintal into 4 panes
tmux split-window -v 

# Send keys to first terminal
tmux send -t main:0.0 "echo 0.0" C-m
tmux send -t main:0.1 "echo 0.1" C-m
#tmux send -t main:0.0 "roslaunch irobot_rosbridge cbridge.launch" C-m
# Send keys to second terminal
#tmux send -t main:0.1 "sleep 2; ${GUI} ${RTF} roslaunch ${ROBOT}_robot ${ENVIRONMENT}.launch ${GAZEBO_OPS}" C-m
# Send keys to third terminal
#tmux send -t main:0.2 "sleep 5; cd ${NAV_DIR}; ${GDB_NAV} ./${NAV} ${NAV_STDOUT} ${NAV_STDERR} ${NAV_OUTPUT}" C-m
# Send keys to fourth terminal
#tmux send -t main:0.3 "sleep 7; cd ${MOB_DIR}; ${GDB_MOB} ./${MOB} ${MOB_STDOUT} ${MOB_STDERR} ${MOB_OUTPUT}" C-m

# Attach to session
tmux -2 attach -t main
