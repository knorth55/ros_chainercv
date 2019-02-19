#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from chainercv.datasets import voc_bbox_label_names
from chainercv.links import SSD300
from chainercv.links import SSD512

from ros_chainercv import ObjectDetectionNode


class SSDObjectDetection(ObjectDetectionNode):

    _models = {
        'ssd300': SSD300,
        'ssd512': SSD512,
    }

    def set_param(self):
        self.model_name = rospy.get_param('~model', 'ssd300')
        self.pretrained_model = rospy.get_param(
            '~pretrained_model', 'voc0712')
        self.label_names = rospy.get_param(
            '~label_names', voc_bbox_label_names)
        self.gpu = rospy.get_param('~gpu', -1)


if __name__ == '__main__':
    rospy.init_node('ssd_object_detection')
    ssd_node = SSDObjectDetection()
    rospy.spin()
