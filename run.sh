#!/bin/sh

gnome-terminal -t "start_ros" -x bash -c "
source devel/setup.bash;
roslaunch rotors_gazebo d435_rgbd.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)
exec bash"
