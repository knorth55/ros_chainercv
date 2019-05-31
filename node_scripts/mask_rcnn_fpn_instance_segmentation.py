#!/usr/bin/env python
import matplotlib
matplotlib.use("Agg")  # NOQA

import rospy

from chainercv.datasets import coco_instance_segmentation_label_names
from chainercv.links import MaskRCNNFPNResNet50
from chainercv.links import MaskRCNNFPNResNet101

from ros_chainercv import InstanceSegmentationNode


class MaskRCNNFPNInstanceSegmentation(InstanceSegmentationNode):

    _models = {
        'mask_rcnn_fpn_resnet50': MaskRCNNFPNResNet50,
        'mask_rcnn_fpn_resnet101': MaskRCNNFPNResNet101,
    }

    def set_param(self):
        self.model_name = rospy.get_param(
            '~model', 'mask_rcnn_fpn_resnet101')
        self.pretrained_model = rospy.get_param(
            '~pretrained_model', 'coco')
        self.label_names = rospy.get_param(
            '~label_names', coco_instance_segmentation_label_names)
        self.gpu = rospy.get_param('~gpu', -1)


if __name__ == '__main__':
    rospy.init_node('mask_rcnn_fpn_instance_segmentation')
    mask_rcnn_fpn_node = MaskRCNNFPNInstanceSegmentation()
    rospy.spin()
