#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from chainercv.datasets import ade20k_semantic_segmentation_label_names
from chainercv.experimental.links import PSPNetResNet101
from chainercv.experimental.links import PSPNetResNet50

from ros_chainercv import SemanticSegmentationNode


class PSPNetSemanticSegmentation(SemanticSegmentationNode):

    _models = {
        'pspnet_resnet101': PSPNetResNet101,
        'pspnet_resnet50': PSPNetResNet50,
    }

    def set_param(self):
        self.model_name = rospy.get_param('~model', 'pspnet_resnet101')
        self.pretrained_model = rospy.get_param(
            '~pretrained_model', 'ade20k')
        self.label_names = rospy.get_param(
            '~label_names', ade20k_semantic_segmentation_label_names)
        self.gpu = rospy.get_param('~gpu', -1)


if __name__ == '__main__':
    rospy.init_node('pspnet_semantic_segmentation')
    pspnet_node = PSPNetSemanticSegmentation()
    rospy.spin()
