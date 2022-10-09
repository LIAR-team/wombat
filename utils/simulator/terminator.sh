#!/bin/bash

gnome-terminal --tab -e "top" --tab -e "htop" &
wait
