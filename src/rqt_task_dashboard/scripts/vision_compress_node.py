#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将 /vision/debug_image 转为 /vision/debug_image/compressed，供 Web/rosbridge 使用。"""

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


class VisionCompressNode(object):
    def __init__(self):
        rospy.init_node('vision_debug_compressed', anonymous=False)
        self._bridge = CvBridge()
        in_topic = rospy.get_param('~in_topic', '/vision/debug_image')
        out_topic = rospy.get_param('~out_topic', '/vision/debug_image/compressed')
        self._quality = int(rospy.get_param('~jpeg_quality', 80))
        self._pub = rospy.Publisher(out_topic, CompressedImage, queue_size=1)
        rospy.Subscriber(in_topic, Image, self._on_image, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo('视觉压缩: %s -> %s (jpeg q=%d)', in_topic, out_topic, self._quality)

    def _on_image(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as err:
            rospy.logwarn_throttle(5.0, 'cv_bridge 失败: %s', err)
            return
        ok, buf = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self._pub.publish(out)


if __name__ == '__main__':
    try:
        VisionCompressNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
