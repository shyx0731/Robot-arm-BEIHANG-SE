#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
根据抓取服务器源码的直接控制测试
向 /wpb_home/mani_ctrl 发送正确的 JointState 消息
控制 lift 和 gripper 两个关节
"""

import rospy
from sensor_msgs.msg import JointState

def test_direct_control():
    """根据抓取服务器源码的控制测试"""
    rospy.init_node('direct_lift_gripper_control')
    
    # 创建发布者
    pub = rospy.Publisher('/wpb_home/mani_ctrl', JointState, queue_size=10)
    
    # 等待发布者建立连接
    rospy.sleep(1.0)
    rospy.loginfo("开始直接控制测试（基于抓取服务器源码）...")
    
    # 关键：使用正确的关节名称
    joint_names = ['lift', 'gripper']  # 不是 elbow_forearm 等！
    
    # 创建消息
    msg = JointState()
    msg.name = joint_names
    msg.position = [0.0, 0.16]  # 初始位置：lift=0, gripper=0.16（打开）
    msg.velocity = [0.5, 5.0]  # 速度：lift=0.5m/s, gripper=5度/秒
    
    rospy.loginfo("测试1: 抬起手臂 (lift 从 0 到 0.2)")
    
    # 测试抬起手臂
    target_lift = 0.2
    steps = 20
    for i in range(steps + 1):
        alpha = i / steps
        current_lift = 0.0 + (target_lift - 0.0) * alpha
        
        msg.position = [current_lift, 0.16]  # gripper保持打开
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        
        rospy.loginfo(f"位置 {i}/{steps}: lift={current_lift:.3f}m, gripper=0.16m")
        rospy.sleep(0.1)
    
    rospy.sleep(2.0)
    
    rospy.loginfo("测试2: 关闭夹爪 (gripper 从 0.16 到 0.03)")
    
    # 测试关闭夹爪
    target_gripper = 0.03
    for i in range(steps + 1):
        alpha = i / steps
        current_gripper = 0.16 + (target_gripper - 0.16) * alpha
        
        msg.position = [target_lift, current_gripper]
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        
        rospy.loginfo(f"位置 {i}/{steps}: lift={target_lift:.3f}m, gripper={current_gripper:.3f}m")
        rospy.sleep(0.1)
    
    rospy.sleep(2.0)
    
    rospy.loginfo("测试3: 抬起手臂更高 (lift 从 0.2 到 0.4)")
    
    # 测试抬起更高
    higher_lift = 0.4
    for i in range(steps + 1):
        alpha = i / steps
        current_lift = target_lift + (higher_lift - target_lift) * alpha
        
        msg.position = [current_lift, target_gripper]
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        
        rospy.loginfo(f"位置 {i}/{steps}: lift={current_lift:.3f}m, gripper={target_gripper:.3f}m")
        rospy.sleep(0.1)
    
    rospy.sleep(2.0)
    
    rospy.loginfo("测试4: 打开夹爪 (gripper 从 0.03 到 0.16)")
    
    # 测试打开夹爪
    for i in range(steps + 1):
        alpha = i / steps
        current_gripper = target_gripper + (0.16 - target_gripper) * alpha
        
        msg.position = [higher_lift, current_gripper]
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        
        rospy.loginfo(f"位置 {i}/{steps}: lift={higher_lift:.3f}m, gripper={current_gripper:.3f}m")
        rospy.sleep(0.1)
    
    rospy.sleep(2.0)
    
    rospy.loginfo("测试5: 降低手臂 (lift 从 0.4 到 0.0)")
    
    # 测试降低手臂
    for i in range(steps + 1):
        alpha = i / steps
        current_lift = higher_lift + (0.0 - higher_lift) * alpha
        
        msg.position = [current_lift, 0.16]
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        
        rospy.loginfo(f"位置 {i}/{steps}: lift={current_lift:.3f}m, gripper=0.16m")
        rospy.sleep(0.1)
    
    rospy.loginfo("测试完成！")

if __name__ == '__main__':
    try:
        test_direct_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")
