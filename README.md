# ROS-ChainerCV

[![GitHub version](https://badge.fury.io/gh/knorth55%2Fros_chainercv.svg)](https://badge.fury.io/gh/knorth55%2Fros_chainercv)
[![Build Status](https://travis-ci.com/knorth55/ros_chainercv.svg?branch=master)](https://travis-ci.com/knorth55/ros_chainercv)
[![Docker Stars](https://img.shields.io/docker/stars/knorth55/ros_chainercv.svg)](https://hub.docker.com/r/knorth55/ros_chainercv)
[![Docker Pulls](https://img.shields.io/docker/pulls/knorth55/ros_chainercv.svg)](https://hub.docker.com/r/knorth55/ros_chainercv)
[![Docker Automated](https://img.shields.io/docker/cloud/automated/knorth55/ros_chainercv.svg)](https://hub.docker.com/r/knorth55/ros_chainercv)
[![Docker Build Status](https://img.shields.io/docker/cloud/build/knorth55/ros_chainercv.svg)](https://hub.docker.com/r/knorth55/ros_chainercv)

This is ROS wrapper of ChainerCV using `catkin_virtualenv`.

This project depends on [locusrobotics/catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv) and [chainer/chainercv](https://github.com/chainer/chainercv).

## Main support

- Ubuntu 18.04 + ROS Melodic

## Docker support

You can get docker images below from Docker hub repository [knorth55/ros_chainercv](https://hub.docker.com/r/knorth55/ros_chainercv).

Available tags are below.
- Ubuntu 16.04 + ROS Kinetic
  - Kinetic: `kinetic-latest`
  - Kinetic + CUDA8.0: `kinetic-cuda80-latest`
  - Kinetic + CUDA9.0: `kinetic-cuda90-latest`
  - Kinetic + CUDA9.1: `kinetic-cuda91-latest`
  - Kinetic + CUDA9.2: `kinetic-cuda92-latest`
  - Kinetic + CUDA10.0: `kinetic-cuda100-latest` 
- Ubuntu 18.04 + ROS Melodic
  - Melodic: `melodic-latest`
  - Melodic + CUDA9.2: `melodic-cuda92-latest`
  - Melodic + CUDA10.0: `melodic-cuda100-latest`

You can get docker images as a command below.
```bash
docker pull knorth55/ros_chainercv:melodic-cuda92-latest
```

## Build

### Build with `nvidia-cuda-toolkit` deb package
Please build this package using the standard build procedure of ROS.

```bash
mkdir catkin_ws/src -p
cd catkin_ws/src
git clone https://github.com/knorth55/ros_chainercv.git
cp ros_chainercv/fc.rosinstall.$ROS_DISTRO .rosinstall
wstool update
rosdep install --ignore-src --from-path src -y -r -i
catkin build
source devel/setup.bash
```

### Build with your own CUDA

```bash
mkdir catkin_ws/src -p
cd catkin_ws/src
git clone https://github.com/knorth55/ros_chainercv.git
cd ros_chaienrcv
vim package.xml
# remove nvidia-cuda dependency
vim requirements.txt
# modify cupy-cuda91 to cupy with your CUDA version
cd ../
cp ros_chainercv/fc.rosinstall.$ROS_DISTRO .rosinstall
wstool update
cd ../
rosdep install --ignore-src --from-path src -y -r -i
catkin build
source devel/setup.bash
```

## Run sample 

### Object Detection 

- FasterRCNN : `roslaunch ros_chainercv sample_faster_rcnn_object_detection.launch`
- FasterRCNN FPN: `roslaunch ros_chainercv sample_faster_rcnn_fpn_object_detection.launch`
- SSD: `roslaunch ros_chainercv sample_ssd_object_detection.launch`
- YOLO: `roslaunch ros_chainercv sample_yolo_object_detection.launch`

### Semantic Segmentation

- PSPNet: `roslaunch ros_chainercv sample_pspnet_semantic_segmentation.launch`
- DeepLab V3+: `roslaunch ros_chainercv sample_deeplab_semantic_segmentation.launch`

### Instance Segmentation

- FCIS: `roslaunch ros_chainercv sample_fcis_instance_segmentation.launch`
- MaskRCNN FPN: `roslaunch ros_chainercv sample_mask_rcnn_fpn_instance_segmentation.launch`

## Tested environment
- Ubuntu: 18.04
- ROS: Melodic
- CUDA: 9.1
- Chainer: 6.0.0
- CuPy: 6.0.0
- ChainerCV: 0.13
