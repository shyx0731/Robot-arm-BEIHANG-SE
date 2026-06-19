#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('grab_trigger_node')
    
    # 建立发布者，完全照抄你 C++ 代码里的话题名！
    pub = rospy.Publisher('/wpb_home/behaviors', String, queue_size=10)
    
    # 【极其关键的一步】：必须停顿 1 秒！等 ROS 网络连接上，否则第一句话会发丢！
    rospy.sleep(1.0) 
    
    # 发送启动暗号
    rospy.loginfo("发送终极抓取指令: 'grab start'")
    pub.publish("grab start")
    
    # 挂起程序
    rospy.spin()
