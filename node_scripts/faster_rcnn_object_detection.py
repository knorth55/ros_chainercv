#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from chainercv.datasets import voc_bbox_label_names
from chainercv.links import FasterRCNNVGG16

from ros_chainercv import ObjectDetectionNode


class FasterRCNNObjectDetection(ObjectDetectionNode):

    _models = {
        'faster_rcnn_vgg16': FasterRCNNVGG16,
    }

    def set_param(self):
        self.model_name = rospy.get_param('~model', 'faster_rcnn_vgg16')
        self.pretrained_model = rospy.get_param(
            '~pretrained_model', 'voc0712')
        self.label_names = rospy.get_param(
            '~label_names', voc_bbox_label_names)
        self.gpu = rospy.get_param('~gpu', -1)


if __name__ == '__main__':
    rospy.init_node('faster_rcnn_object_detection')
    faster_rcnn_node = FasterRCNNObjectDetection()
    rospy.spin()
