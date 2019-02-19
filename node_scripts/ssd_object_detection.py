#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy
import sys

import chainer
from chainercv.datasets import voc_bbox_label_names
from chainercv.links import SSD300
from chainercv.links import SSD512

from ros_chainercv import ObjectDetectionNode


class SSDObjectDetection(ObjectDetectionNode):

    def __init__(self):
        model_name = rospy.get_param('~model', 'ssd300')
        pretrained_model = rospy.get_param('~pretrained_model', 'voc0712')
        self.label_names = rospy.get_param(
            '~label_names', voc_bbox_label_names)
        gpu = rospy.get_param('~gpu', -1)

        if model_name == 'ssd300':
            cls = SSD300
        elif model_name == 'ssd512':
            cls = SSD512
        else:
            rospy.logerr('model is not supported: {}'.format(model_name))
            sys.exit()

        self.model = cls(
            n_fg_class=len(self.label_names),
            pretrained_model=pretrained_model)

        if gpu >= 0:
            chainer.cuda.get_device_from_id(gpu).use()
            self.model.to_gpu()

        super(SSDObjectDetection, self).__init__()


if __name__ == '__main__':
    rospy.init_node('ssd_object_detection')
    ssd_node = SSDObjectDetection()
    rospy.spin()
