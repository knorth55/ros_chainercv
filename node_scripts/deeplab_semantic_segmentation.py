#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from chainercv.datasets import ade20k_semantic_segmentation_label_names
from chainercv.links import DeepLabV3plusXception65

from ros_chainercv import SemanticSegmentationNode


class DeepLabSemanticSegmentation(SemanticSegmentationNode):

    _models = {
        'deeplab_v3plus_xception65': DeepLabV3plusXception65,
    }

    def set_param(self):
        self.model_name = rospy.get_param(
            '~model', 'deeplab_v3plus_xception65')
        self.pretrained_model = rospy.get_param(
            '~pretrained_model', 'ade20k')
        self.label_names = rospy.get_param(
            '~label_names', ade20k_semantic_segmentation_label_names)
        self.gpu = rospy.get_param('~gpu', -1)


if __name__ == '__main__':
    rospy.init_node('deeplab_semantic_segmentation')
    deeplab_node = DeepLabSemanticSegmentation()
    rospy.spin()
