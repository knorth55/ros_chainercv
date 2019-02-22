FROM osrf/ros:melodic-desktop-bionic

ENV ROS_DISTRO=melodic

RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# FIXME: https://github.com/start-jsk/jsk_apc/pull/2664
RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y \
    wget \
    git \
    python-catkin-tools \
    python-pip \
    python-rosdep \
    python-setuptools \
    python-wstool 

RUN mkdir -p ~/ros/ros_chainercv_ws/src && \
    cd ~/ros/ros_chainercv_ws/src && \
    wstool init && \
    wstool set knorth55/ros_chainercv https://github.com/knorth55/ros_chainercv.git -v master --git -y && \
    wstool up -j 2

RUN cd ~/ros/ros_chainercv_ws/src && \
    rosdep update && \
    rosdep install --rosdistro $ROS_DISTRO -r -y -i --from-paths . 

RUN cd ~/ros/ros_chainercv_ws && \
    source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build
