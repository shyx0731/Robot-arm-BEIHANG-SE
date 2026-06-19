#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

class WalleArm:
    def __init__(self):
        # 建立发布者，直接夺取最底层的关节控制权！
        self.pub = rospy.Publisher('/wpb_home/mani_ctrl', JointState, queue_size=10)
        rospy.sleep(1.0) # 等待连接
        
    def set_arm(self, lift_height, gripper_width):
        """
        控制机械臂的万能函数
        :param lift_height: 升降高度 (米)，例如 0.5
        :param gripper_width: 手爪指间距 (米)，打开是0.16，闭合一般是 0.05 到 0 之间
        """
        msg = JointState()
        msg.name = ["lift", "gripper"] # 严格按照源码里要求的名字
        
        # 设置目标位置
        msg.position = [lift_height, gripper_width]
        
        # 设置运动速度 (你可以调大调小)
        msg.velocity = [0.1, 5.0] # lift速度0.1米/秒，夹爪开合角速度5度/秒
        
        # 发送底层指令
        self.pub.publish(msg)
        rospy.loginfo(f"机械臂动作 -> 高度: {lift_height}m, 夹爪宽度: {gripper_width}m")

# ================= 使用演示 =================
if __name__ == '__main__':
    rospy.init_node('my_arm_test_node')
    arm = WalleArm()
    
    # 模拟你们未来抓纸箱的动作序列：
    
    # 1. 初始状态：手臂抬到 0.4米，夹爪完全张开准备
    arm.set_arm(lift_height=0.4, gripper_width=0.16)
    rospy.sleep(3) # 等它转到位
    
    # 2. (假设此时轮子已经开到纸箱面前了) 夹爪闭合，抓住纸箱！
    arm.set_arm(lift_height=0.4, gripper_width=0.03) # 0.03 表示抓紧
    rospy.sleep(2)
    
    # 3. 把纸箱举高高到 0.8米，准备运走
    arm.set_arm(lift_height=0.8, gripper_width=0.03)
    rospy.sleep(3)
