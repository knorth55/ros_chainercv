import matplotlib.pyplot as plt
import numpy as np
import rospy
import sys

import chainer
from chainercv.experimental.links import PSPNetResNet101
from chainercv.experimental.links import PSPNetResNet50
from chainercv.visualizations import vis_image
from chainercv.visualizations import vis_semantic_segmentation
from cv_bridge import CvBridge
from jsk_topic_tools import ConnectionBasedTransport

from sensor_msgs.msg import Image


class SemanticSegmentationNode(ConnectionBasedTransport):

    _models = {
        'pspnet_resnet101': PSPNetResNet101,
        'pspnet_resnet50': PSPNetResNet50,
    }

    def __init__(self):
        super(SemanticSegmentationNode, self).__init__()
        self.set_param()
        self.check_param()
        self.set_model()

        if self.gpu >= 0:
            self.model.to_gpu(self.gpu)

        self.cv_bridge = CvBridge()

        # advertise
        self.pub_label = self.advertise(
            '~output/label', Image, queue_size=1)
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
        self.model = model_cls(
            n_class=len(self.label_names),
            pretrained_model=self.pretrained_model)

    def subscribe(self):
        self.sub_image = rospy.Subscriber(
            '~input/image', Image, self.image_cb,
            queue_size=1, buff_size=2**26)

    def unsubscribe(self):
        self.sub_image.unregister()

    def image_cb(self, msg):
        chainer.cuda.get_device_from_id(self.gpu).use()

        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        img = img.transpose((2, 0, 1)).astype(np.float32)

        labels = self.model.predict([img])
        label = labels[0]

        # label
        label_msg = self.cv_bridge.cv2_to_imgmsg(
            label.astype(np.int32), '32SC1')
        label_msg.header = msg.header

        # visualize
        vis_msg = self.visualize(img, label)
        vis_msg.header = msg.header

        # publish
        self.pub_label.publish(label_msg)
        self.pub_vis.publish(vis_msg)

    def visualize(self, img, label):
        fig = plt.gcf()
        ax1 = fig.add_subplot(1, 2, 1)
        vis_image(img, ax=ax1)
        ax2 = fig.add_subplot(1, 2, 2)
        ax2, legend_handles = vis_semantic_segmentation(
            img, label, self.label_names, ax=ax2)
        ax2.legend(
            handles=legend_handles, bbox_to_anchor=(1, 1), loc=2)
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        vis_img = np.fromstring(
            fig.canvas.tostring_rgb(), dtype=np.uint8)
        fig.clf()
        vis_img.shape = (h, w, 3)
        plt.close()
        vis_msg = self.cv_bridge.cv2_to_imgmsg(vis_img, "rgb8")
        return vis_msg
