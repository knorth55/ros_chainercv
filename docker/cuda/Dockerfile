ARG CUDA_MAJOR_VERSION
ARG CUDA_MINOR_VERSION
ARG UBUNTU_VERSION
FROM nvidia/cuda:${CUDA_MAJOR_VERSION}.${CUDA_MINOR_VERSION}-cudnn7-devel-ubuntu${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y --no-install-recommends \
    git \
    lsb-release \
    sed \
    wget \
    gnupg2

RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN wget http://packages.ros.org/ros.key -O - | apt-key add -

ARG ROS_DISTRO
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
    wstool set locusrobotics/catkin_virtualenv https://github.com/locusrobotics/catkin_virtualenv.git -v 0.4.1 --git -y && \
    wstool up -j 2

ARG CUDA_MAJOR_VERSION
ARG CUDA_MINOR_VERSION
RUN cd ~/ros/ros_chainercv_ws/src/knorth55/ros_chainercv && \
    sed -e 's/.*nvidia-cuda.*//g' -i package.xml && \
    sed -e 's/cuda91/cuda'"${CUDA_MAJOR_VERSION}${CUDA_MINOR_VERSION}"'/g' -i requirements.txt

RUN cd ~/ros/ros_chainercv_ws/src && \
    rosdep init && \
    rosdep update && \
    rosdep install --rosdistro ${ROS_DISTRO} -r -y -i --from-paths .

RUN cd ~/ros/ros_chainercv_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build

RUN rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
