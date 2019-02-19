#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from ros_chainercv import ObjectDetectionNode


if __name__ == '__main__':
    rospy.init_node('object_detection')
    object_detection = ObjectDetectionNode()
    rospy.spin()
