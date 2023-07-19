#!/bin/bash
current_path=$(pwd)

source devel/setup.bash

# catkin_make

if [ $? -ne 0 ]; then
    echo ""
    echo -e "\033[31m!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
    echo -e "\033[31mMAKE FAILED\033[0m"
    echo -e "\033[31m!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
    exit
fi

# echo "Running in real mode"

# if the first argument is 1 then run in real mod
if [ "$1" = "1" ]; then
    echo "Running in real mode"
    roslaunch master master.launch is_urban:=1 vel:=10
else 
    echo "Running in race mode with vel = $1"
    roslaunch master master.launch is_urban:=0 vel:=$1
fi