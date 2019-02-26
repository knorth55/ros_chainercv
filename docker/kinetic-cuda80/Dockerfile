FROM nvidia/cuda:8.0-cudnn7-devel-ubuntu16.04

ENV ROS_DISTRO=kinetic
ENV CUPY_CUDA_VERSION=80

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y --no-install-recommends \
    git \
    lsb-release \
    sed \
    wget

RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-core \
    python-catkin-tools \
    python-pip \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-setuptools \
    python-wstool

RUN mkdir -p ~/ros/ros_chainercv_ws/src && \
    cd ~/ros/ros_chainercv_ws/src && \
    wstool init && \
    wstool set knorth55/ros_chainercv https://github.com/knorth55/ros_chainercv.git -v master --git -y && \
    wstool up -j 2

RUN cd ~/ros/ros_chainercv_ws/src/knorth55/ros_chainercv && \
    sed -e 's/.*nvidia-cuda.*//g' -i package.xml && \
    sed -e 's/cuda91/cuda${CUPY_CUDA_VERSION}/g' -i requirements.txt

RUN cd ~/ros/ros_chainercv_ws/src && \
    rosdep init && \
    rosdep update && \
    rosdep install --rosdistro $ROS_DISTRO -r -y -i --from-paths .

RUN cd ~/ros/ros_chainercv_ws && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build

RUN rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
