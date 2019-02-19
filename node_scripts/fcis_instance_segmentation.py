#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from chainercv.datasets import coco_instance_segmentation_label_names
from chainercv.experimental.links import FCISResNet101

from ros_chainercv import InstanceSegmentationNode


class FCISInstanceSegmentation(InstanceSegmentationNode):

    _models = {
        'fcis_resnet101': FCISResNet101,
    }

    def set_param(self):
        self.model_name = rospy.get_param('~model', 'fcis_resnet101')
        self.pretrained_model = rospy.get_param(
            '~pretrained_model', 'coco')
        self.label_names = rospy.get_param(
            '~label_names', coco_instance_segmentation_label_names)
        self.gpu = rospy.get_param('~gpu', -1)


if __name__ == '__main__':
    rospy.init_node('fcis_semantic_segmentation')
    fcis_node = FCISInstanceSegmentation()
    rospy.spin()
