#!/bin/bash
current_path=$(pwd)

catkin_make

if [ $? -ne 0 ]; then
    echo ""
    echo -e "\033[31m!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
    echo -e "\033[31mMAKE FAILED\033[0m"
    echo -e "\033[31m!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\033[0m"
    exit
fi

echo "Running in real mode"
source $current_path/devel/setup.bash
roslaunch master master.launch