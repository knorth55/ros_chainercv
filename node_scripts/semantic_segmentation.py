#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from ros_chainercv import SemanticSegmentationNode


if __name__ == '__main__':
    rospy.init_node('semantic_segmentation')
    semantic_segmentation = SemanticSegmentationNode()
    rospy.spin()
