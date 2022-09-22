########## Pull ##########
FROM ros:noetic
########## Non-interactive ##########
ENV DEBIAN_FRONTEND=noninteractive
########## Common tool ##########
RUN apt-get update && \
	apt-get install -y \
		vim \
		wget \
		unzip \
		git \
        python-tk
########## ROS setup ##########
RUN mkdir -p ~/catkin_ws/src && \
	cd ~/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make" && \
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=~/catkin_ws" >> ~/.bashrc
## cmk
RUN echo 'function cmk(){ \n\
		lastpwd=$OLDPWD \n\
		cpath=$(pwd) \n\
		cd ~/catkin_ws \n\
		catkin_make $@ \n\
		cd $cpath \n\
		OLDPWD=$lastpwd \n\
        }' >> ~/.bashrc
########## pc_normal_estimation ##########
RUN apt-get update && \
    apt-get install -y \
		libpcl-dev \
		ros-noetic-pcl-conversions \
		ros-noetic-rviz && \
	cd ~/catkin_ws/src && \
	git clone https://github.com/ozakiryota/pc_normal_estimation.git && \
	cd ~/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"
########## Velodyne driver ##########
RUN apt-get update && \
	apt-get install -y \
		ros-noetic-pcl-ros \
		ros-noetic-diagnostics \
		ros-noetic-roslint \
		ros-noetic-angles \
		libyaml-cpp-dev \
		libpcap-dev && \
	cd ~/catkin_ws/src && \
	git clone https://github.com/ros-drivers/velodyne.git && \
	cd ~/catkin_ws/ && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"
########## Initial position ##########
WORKDIR /root/catkin_ws/src/pc_normal_estimation