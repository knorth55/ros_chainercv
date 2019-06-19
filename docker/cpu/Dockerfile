ARG ROS_DISTRO
ARG UBUNTU_NAME
FROM osrf/ros:${ROS_DISTRO}-desktop-${UBUNTU_NAME}

ARG ROS_DISTRO
RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install -y wget git
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -

RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y \
    python-catkin-tools \
    python-pip \
    python-rosdep \
    python-setuptools \
    python-wstool

RUN mkdir -p ~/ros/ros_chainercv_ws/src && \
    cd ~/ros/ros_chainercv_ws/src && \
    wstool init && \
    wstool set knorth55/ros_chainercv https://github.com/knorth55/ros_chainercv.git -v master --git -y && \
    wstool set locusrobotics/catkin_virtualenv https://github.com/locusrobotics/catkin_virtualenv.git -v 0.3.0 --git -y && \
    wstool up -j 2

RUN cd ~/ros/ros_chainercv_ws/src/knorth55/ros_chainercv && \
    sed -e 's/.*nvidia-cuda.*//g' -i package.xml && \
    sed -e 's/cupy-cuda91.*//g' -i requirements.txt

RUN cd ~/ros/ros_chainercv_ws/src && \
    rosdep update && \
    rosdep install --rosdistro ${ROS_DISTRO} -r -y -i --from-paths .

RUN cd ~/ros/ros_chainercv_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build

RUN rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*
