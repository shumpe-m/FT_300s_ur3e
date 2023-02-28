FROM nvidia/opengl:base-ubuntu20.04
ENV DEBIAN_FRONTEND=noninteractive


########################
### INSTALL PACKAGES ###
########################
RUN apt-get update && apt-get install -y \
    sudo \
    vim \
    wget \
    unzip \
    git \
    python3-tk \
    python3-pip


###################
### ROS noetic ###
###################
RUN apt-get update && apt-get install -y \
    lsb-release \
    curl \
    gnupg2 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
	ros-noetic-desktop-full \
	&& echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc


#####################
### INSTALL CMAKE ###
#####################
# Reference: https://apt.kitware.com/
RUN git clone https://gitlab.kitware.com/cmake/cmake.git \
	&& cd cmake \
	&& git checkout tags/v3.16.3 \
	&& ./bootstrap --parallel=8 \
	&& make -j8 \
	&& make install \
	&& cd .. \
	&& rm -rf cmake

###########################################
### INSTALL Universal_Robots_ROS_Driver ###
###########################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rospkg \
    python3-catkin-pkg \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-rosdep \
    python3-catkin-tools \
    ros-noetic-moveit-commander \
    python3 \
    ros-noetic-universal-robots

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"
RUN mkdir -p /root/catkin_ws/src
RUN cd /root/catkin_ws \
    && git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver


##############
### ur pkg ###
##############
RUN cd /root/catkin_ws \
    && git clone -b melodic-devel-2 https://github.com/shumpe-m/ur3.git src/ros_ur3 \
    && git clone  -b noetic https://github.com/shumpe-m/FT_300s_ur3e.git src/FT_300s_ur3e

RUN cd /root/catkin_ws \
    && rosinstall /root/catkin_ws/src /opt/ros/noetic src/ros_ur3/dependencies.rosinstall \
    && apt-get update -qq \
    && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=noetic -y \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /root/catkin_ws; catkin_make" \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/root/catkin_ws" >> ~/.bashrc \
    && echo "export ROS_WORKSPACE=/root/catkin_ws" >> ~/.bashrc \
    && echo "chmod -R u+x /root/catkin_ws/src/FT_300s_ur3e/ur_control_scripts/scripts" >> ~/.bashrc


RUN echo "cd /root/catkin_ws" >> ~/.bashrc