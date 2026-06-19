#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

class VisionNode:
    def __init__(self):
        rospy.init_node('walle_vision_node')
        self.bridge = CvBridge()
        self.depth_image = None
        
        # 订阅高清彩图和深度图
        self.sub_rgb = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.rgb_cb)
        self.sub_depth = rospy.Subscriber("/kinect2/hd/image_depth_rect", Image, self.depth_cb)
        
        # 往你的微服务总线发布目标
        self.target_pub = rospy.Publisher("/walle/target_info", Pose, queue_size=1)
        rospy.loginfo("[侦察兵] 视觉节点已上线，开始扫描...")

    def depth_cb(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except:
            pass

    def rgb_cb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 找绿色瓶子(模拟本色纸箱)
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        contours_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours_g:
            best_contour = max(contours_g, key=cv2.contourArea)
            if cv2.contourArea(best_contour) > 500:
                x, y, w, h = cv2.boundingRect(best_contour)
                cx = x + w // 2
                cy = y + h // 2
                
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # 【修复核心 Bug】：查出真实的距离(米)，然后再发给总司令！
                if self.depth_image is not None:
                    distance = self.depth_image[cy, cx]
                    
                    if not np.isnan(distance) and 0.3 < distance < 2.0:
                        target = Pose()
                        target.position.x = distance  # 真实的物理距离！不再是像素了！
                        target.position.z = 0.0       # 0.0 代表绿色/本色
                        self.target_pub.publish(target)

        cv2.imshow("Wall-E Vision", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    v = VisionNode()
    rospy.spin()
