FROM osrf/ros:melodic-desktop-full

WORKDIR /root/

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
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
    iputils-ping net-tools \
    python-tk \
    python-pip

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
    python-rospkg \
    python-catkin-pkg \
    python-rosinstall \
    python-rosinstall-generator \
    python-rosdep \
    python-catkin-tools \
    ros-melodic-moveit-commander \
    python \
    ros-melodic-universal-robots \
    build-essential

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"
RUN mkdir -p /root/catkin_ws/src
RUN cd /root/catkin_ws \
    && git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver 


##############
### ur pkg ###
##############
RUN cd /root/catkin_ws \
    && git clone -b melodic-devel-2 https://github.com/shumpe-m/ur3.git src/ros_ur3 \
    && git clone  -b melodic https://github.com/shumpe-m/FT_300s_ur3e.git src/FT_300s_ur3e

RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
RUN cd /root/catkin_ws \
    && rosinstall /root/catkin_ws/src /opt/ros/melodic src/ros_ur3/dependencies.rosinstall \
    && apt-get update -qq \
    && rosdep init && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=melodic -y \
    && /bin/bash -c "source /opt/ros/melodic/setup.bash; cd /root/catkin_ws; catkin_make" \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/root/catkin_ws" >> ~/.bashrc \
    && echo "export ROS_WORKSPACE=/root/catkin_ws" >> ~/.bashrc \
    && echo "chmod -R u+x /root/catkin_ws/src/FT_300s_ur3e/ur_control_scripts/scripts" >> ~/.bashrc


RUN echo "cd /root/catkin_ws" >> ~/.bashrc