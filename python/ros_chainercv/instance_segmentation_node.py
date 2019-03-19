import matplotlib.pyplot as plt
import numpy as np
import rospy
import sys

import chainer
from chainercv.experimental.links import FCISResNet101
from chainercv.utils import mask_to_bbox
from chainercv.visualizations.colormap import voc_colormap
from chainercv.visualizations import vis_bbox
from chainercv.visualizations import vis_instance_segmentation
from cv_bridge import CvBridge
from jsk_topic_tools import ConnectionBasedTransport

from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from sensor_msgs.msg import Image


class InstanceSegmentationNode(ConnectionBasedTransport):

    _models = {
        'fcis_resnet101': FCISResNet101,
    }

    def __init__(self):
        super(InstanceSegmentationNode, self).__init__()
        self.set_param()
        self.check_param()
        self.set_model()

        if self.gpu >= 0:
            self.model.to_gpu(self.gpu)

        self.cv_bridge = CvBridge()

        # advertise
        self.pub_rects = self.advertise(
            '~output/rects', RectArray, queue_size=1)
        self.pub_label = self.advertise(
            '~output/label', Image, queue_size=1)
        self.pub_inst_label = self.advertise(
            '~output/inst_label', Image, queue_size=1)
        self.pub_class = self.advertise(
            '~output/class', ClassificationResult, queue_size=1)
        self.pub_vis = self.advertise(
            '~output/vis', Image, queue_size=1)

    def set_param(self):
        self.model_name = rospy.get_param('~model')
        self.pretrained_model = rospy.get_param('~pretrained_model')
        self.label_names = rospy.get_param('~label_names')
        self.gpu = rospy.get_param('~gpu')

    def check_param(self):
        if self.model_name not in self._models.keys():
            rospy.logerr('model is not supported: {}'.format(self.model_name))
            sys.exit(1)

    def set_model(self):
        model_cls = self._models[self.model_name]
        if self.model_name == 'fcis_resnet101' \
                and self.pretrained_model == 'coco':
            proposal_creator_params = model_cls.proposal_creator_params
            proposal_creator_params['min_size'] = 2
            self.model = model_cls(
                n_fg_class=len(self.label_names),
                anchor_scales=(4, 8, 16, 32),
                pretrained_model=self.pretrained_model,
                proposal_creator_params=proposal_creator_params)
        else:
            self.model = model_cls(
                n_fg_class=len(self.label_names),
                pretrained_model=self.pretrained_model)

    def subscribe(self):
        self.sub_image = rospy.Subscriber(
            '~input/image', Image, self.image_cb,
            queue_size=1, buff_size=2**26)

    def unsubscribe(self):
        self.sub_image.unregister()

    def image_cb(self, msg):
        if self.gpu >= 0:
            chainer.cuda.get_device_from_id(self.gpu).use()

        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        img = img.transpose((2, 0, 1)).astype(np.float32)

        masks, labels, scores = self.model.predict([img])
        mask, label, score = masks[0], labels[0], scores[0]
        bbox = mask_to_bbox(mask)

        # bbox
        rects_msg = RectArray(header=msg.header)
        for bb in bbox:
            rect = Rect(
                x=bb[1], y=bb[0], width=bb[3] - bb[1], height=bb[2] - bb[0])
            rects_msg.rects.append(rect)

        lbl_img = - np.ones(img.shape[1:], dtype=np.int32)
        inst_lbl_img = - np.ones(img.shape[1:], dtype=np.int32)
        for inst_lbl, (msk, lbl) in enumerate(zip(mask, label)):
            lbl_img[msk] = lbl
            inst_lbl_img[msk] = inst_lbl

        # label
        label_msg = self.cv_bridge.cv2_to_imgmsg(
            lbl_img.astype(np.int32), '32SC1')
        label_msg.header = msg.header

        # inst label
        inst_label_msg = self.cv_bridge.cv2_to_imgmsg(
            inst_lbl_img.astype(np.int32), '32SC1')
        inst_label_msg.header = msg.header

        # classification
        cls_msg = ClassificationResult(
            header=msg.header,
            classifier=self.model.__class__.__name__,
            target_names=self.label_names,
            labels=label,
            label_names=[self.label_names[lbl] for lbl in label],
            label_proba=score
        )

        # visualize
        vis_msg = self.visualize(img, mask, bbox, label, score)
        vis_msg.header = msg.header

        # publish
        self.pub_rects.publish(rects_msg)
        self.pub_label.publish(label_msg)
        self.pub_inst_label.publish(inst_label_msg)
        self.pub_class.publish(cls_msg)
        self.pub_vis.publish(vis_msg)

    def visualize(self, img, mask, bbox, label, score):
        colors = voc_colormap(list(range(1, len(mask) + 1)))
        ax = vis_bbox(
            img, bbox, instance_colors=colors, alpha=0.5, linewidth=1.5)
        vis_instance_segmentation(
            None, mask, label, score, self.label_names,
            instance_colors=colors, alpha=0.7, ax=ax)
        fig = plt.gcf()
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        vis_img = np.fromstring(
            fig.canvas.tostring_rgb(), dtype=np.uint8)
        fig.clf()
        vis_img.shape = (h, w, 3)
        plt.close()
        vis_msg = self.cv_bridge.cv2_to_imgmsg(vis_img, "rgb8")
        return vis_msg
