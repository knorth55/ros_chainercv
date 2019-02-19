#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from chainercv.datasets import voc_bbox_label_names
from chainercv.experimental.links import YOLOv2Tiny
from chainercv.links import YOLOv2
from chainercv.links import YOLOv3

from ros_chainercv import ObjectDetectionNode


class YOLOObjectDetection(ObjectDetectionNode):

    _models = {
        'yolo_v2': YOLOv2,
        'yolo_v2_tiny': YOLOv2Tiny,
        'yolo_v3': YOLOv3,
    }

    def set_param(self):
        self.model_name = rospy.get_param('~model', 'yolo_v3')
        self.pretrained_model = rospy.get_param(
            '~pretrained_model', 'voc0712')
        self.label_names = rospy.get_param(
            '~label_names', voc_bbox_label_names)
        self.gpu = rospy.get_param('~gpu', -1)


if __name__ == '__main__':
    rospy.init_node('yolo_object_detection')
    yolo_node = YOLOObjectDetection()
    rospy.spin()
