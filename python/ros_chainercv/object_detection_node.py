import matplotlib.pyplot as plt
import numpy as np
import rospy

from chainercv.visualizations import vis_bbox
from cv_bridge import CvBridge
from jsk_topic_tools import ConnectionBasedTransport

from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from sensor_msgs.msg import Image


class ObjectDetectionNode(ConnectionBasedTransport):

    def __init__(self):
        super(ObjectDetectionNode, self).__init__()
        self.cv_bridge = CvBridge()

        # advertise
        self.pub_rects = self.advertise(
            '~output/rects', RectArray, queue_size=1)
        self.pub_class = self.advertise(
            '~output/class', ClassificationResult, queue_size=1)
        self.pub_vis = self.advertise(
            '~output/vis', Image, queue_size=1)

    def subscribe(self):
        self.sub_image = rospy.Subscriber(
            '~input/image', Image, self.image_cb,
            queue_size=1, buff_size=2**26)

    def unsubscribe(self):
        self.sub_image.unregister()

    def image_cb(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        img = img.transpose((2, 0, 1))

        bboxes, labels, scores = self.model.predict([img])
        bbox, label, score = bboxes[0], labels[0], scores[0]

        # bbox
        rect_msg = RectArray(header=msg.header)
        for bb in bbox:
            rect = Rect(
                x=bb[1], y=bb[0], width=bb[3] - bb[1], height=bb[2] - bb[0])
            rect_msg.rects.append(rect)

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
        vis_msg = self.visualize(img, bbox, label, score)
        vis_msg.header = msg.header

        # publish
        self.pub_rects.publish(rect_msg)
        self.pub_class.publish(cls_msg)
        self.pub_vis.publish(vis_msg)

    def visualize(self, img, bbox, label, score):
        vis_bbox(img, bbox, label, score, self.label_names)
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
