#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class BoxDetectorAndGrabber:
    def __init__(self):
        rospy.init_node('box_grabber_node', anonymous=True)
        self.bridge = CvBridge()
        self.latest_depth_image = None
        
        self.is_grabbing = False 
        self.stable_count = 0

        self.image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, self.depth_callback)
        
        self.grab_action_pub = rospy.Publisher('/wpb_home/grab_action', Pose, queue_size=1)
        
        rospy.loginfo("视觉抓取节点(3D坐标版)已启动，正在寻找目标...")

    def depth_callback(self, data):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

    def rgb_callback(self, data):
        if self.is_grabbing:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 寻找蓝色箱子
        #lower_blue = np.array([100, 50, 50])
        #upper_blue = np.array([130, 255, 255])
        #mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        lower_green = np.array([35, 50, 50])    # 较低的绿色阈值
        upper_green = np.array([85, 255, 255])  # 较高的绿色阈值
        
        # 创建绿色掩码
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cx = x + w // 2
                cy = y + h // 2

                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

                if self.latest_depth_image is not None:
                    color_h, color_w = cv_image.shape[:2]
                    depth_h, depth_w = self.latest_depth_image.shape[:2]

                    scale_x = depth_w / color_w
                    scale_y = depth_h / color_h

                    depth_x = min(max(int(cx * scale_x), 0), depth_w - 1)
                    depth_y = min(max(int(cy * scale_y), 0), depth_h - 1)

                    distance = self.latest_depth_image[depth_y, depth_x]
                    
                    if not np.isnan(distance) and distance > 0:
                        # ==================================================
                        # 【核心魔法：将像素和距离，转换成机器人眼里的 3D 坐标】
                        # ==================================================
                        # 机器人的 X 轴是正前方 (也就是测出来的 distance)
                        robot_x = distance
                        
                        # 机器人的 Y 轴是左右。彩色画面宽度的一半是 960。
                        # 利用相似三角形粗略计算左右偏移的物理距离 (假设视角大约 70度)
                        pixel_offset_x = 960 - cx 
                        robot_y = pixel_offset_x * distance * 0.001 
                        
                        # 机器人的 Z 轴是高度。这里我们先不管它，底层的抓取程序通常会自动往下摸索
                        robot_z = 0.0 

                        rospy.loginfo(f"真实3D坐标: 前方 {robot_x:.2f}米, 偏左 {robot_y:.2f}米")
                        
                        if 0.5 < distance < 1.5:
                            self.stable_count += 1
                            if self.stable_count > 3:
                                # 注意！这里传进去的是算好的物理米数，不是像素了！
                                self.execute_grab(robot_x, robot_y, robot_z)
                        else:
                            self.stable_count = 0

        cv2.imshow("Box Grabber", cv_image)
        cv2.waitKey(1)

    def execute_grab(self, px, py, pz):
        rospy.logwarn(f">>> 开始抓取！目标物理位置：X={px:.2f}, Y={py:.2f} <<<")
        self.is_grabbing = True # 锁死，让摄像头不再重复发指令
        
        target_msg = Pose()
        target_msg.position.x = px
        target_msg.position.y = py
        target_msg.position.z = pz
        
            # 直接发布给底层抓取动作节点
        for _ in range(3):
            self.grab_action_pub.publish(target_msg)
            rospy.sleep(0.1)
	    
        rospy.loginfo("抓取坐标已发送至底层 Action 节点！")

if __name__ == '__main__':
    try:
        detector = BoxDetectorAndGrabber()
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
