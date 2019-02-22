# ROS-ChainerCV

[![GitHub version](https://badge.fury.io/gh/knorth55%2Fros_chainercv.svg)](https://badge.fury.io/gh/knorth55%2Fros_chainercv)
[![Build Status](https://travis-ci.com/knorth55/ros_chainercv.svg?branch=master)](https://travis-ci.com/knorth55/ros_chainercv)

This is ROS wrapper of ChainerCV using `catkin_virtualenv`.

This project depends on [locusrobotics/catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv) and [chainer/chainercv](https://github.com/chainer/chainercv).

## Support

- Ubuntu 18.04 + ROS Melodic

## Build

### Build with `nvidia-cuda-toolkit` deb package
Please build this package using the standard build procedure of ROS.

```bash
mkdir catkin_ws/src -p
cd catkin_ws/src
git clone https://github.com/knorth55/ros_chainercv.git
cd ..
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
cd ../../
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

### Instance Segmentation

- FCIS: `roslaunch ros_chainercv sample_fcis_instance_segmentation.launch`

## Test deependency version
- Ubuntu: 18.04
- ROS: Melodic
- CUDA: 9.1
- Chainer: 5.2.0
- CuPy: 5.2.0
- ChainerCV: 0.12
