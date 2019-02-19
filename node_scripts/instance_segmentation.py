#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from ros_chainercv import InstanceSegmentationNode


if __name__ == '__main__':
    rospy.init_node('instance_segmentation')
    instance_segmentation = InstanceSegmentationNode()
    rospy.spin()
