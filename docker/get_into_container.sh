#!/bin/bash

xhost +

image="pc_normal_estimation"
tag="latest"

docker run -it --rm \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v $(pwd)/..:/root/catkin_ws/src/$image \
	-v $HOME/rosbag:/root/rosbag \
	$image:$tag