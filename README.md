# ROS-ChainerCV

This is ROS wrapper of ChainerCV using `catkin_virtualenv`.

This project depends on [locusrobotics/catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv).

## Build 

Please build this package using the standard build procedure of ROS.

```bash
mkdir catkin_ws/src -p
cd catkin_ws/src
git clone https://github.com/locusrobotics/catkin_virtualenv.git
git clone https://github.com/knorth55/ros_chainercv.git
cd ..
catkin build
source devel/setup.bash
```

## Run sample 

### Object Detection 

*FasterRCNN*

```bash
roslaunch ros_chainercv sample_faster_rcnn_object_detection.launch gpu:=-1
```

*FasterRCNN FPN*

```bash
roslaunch ros_chainercv sample_faster_rcnn_fpn_object_detection.launch gpu:=-1
```

*SSD*

```bash
roslaunch ros_chainercv sample_ssd_object_detection.launch gpu:=-1
```

*YOLO*

```bash
roslaunch ros_chainercv sample_yolo_object_detection.launch gpu:=-1
```
