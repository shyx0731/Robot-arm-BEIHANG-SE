#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WPB Home 机械臂控制 FSM - 稳定版
"""

import rospy
import math
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class WalleGrabFSM:
    def __init__(self):
        rospy.init_node('walle_grab_fsm')
        
        # ==================== 状态定义 ====================
        self.STATE_IDLE = 0
        self.STATE_APPROACH = 1      # 靠近目标
        self.STATE_LOWER_ARM = 2     # 降臂
        self.STATE_GRAB = 3          # 抓取
        self.STATE_LIFT = 4          # 抬起
        self.STATE_RETREAT = 5       # 后退
        self.STATE_DONE = 6          # 完成
        
        self.state = self.STATE_IDLE
        
        # ==================== 参数 ====================
        self.target_distance = 0.0
        self.target_height = 0.0
        
        # 初始位置
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        
        # ==================== 发布器 ====================
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.arm_pub = rospy.Publisher('/wpb_home/mani_ctrl', JointState, queue_size=5)
        
        # ==================== 订阅器 ====================
        rospy.Subscriber("/walle/target_info", Pose, self.target_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # 机械臂消息模板
        self.arm_msg = JointState()
        self.arm_msg.name = ["lift", "gripper"]
        self.arm_msg.position = [0.08, 0.16]  # 初始：降臂+开爪
        self.arm_msg.velocity = [0.2, 5.0]    # 速度
        self.arm_msg.effort = []              # 空数组
        
        # 初始化时发一次，确保机械臂在准备位置
        rospy.sleep(2)  # 等待发布者注册
        rospy.loginfo("初始化机械臂位置...")
        self.arm_msg.header.stamp = rospy.Time.now()
        self.arm_pub.publish(self.arm_msg)
        
        rospy.loginfo("FSM 就绪，等待目标...")
    
    def odom_callback(self, msg):
        """里程计回调，获取当前位置"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    def target_callback(self, msg):
        """目标回调"""
        if self.state != self.STATE_IDLE:
            rospy.logwarn("忙碌中，忽略新目标")
            return
        
        self.target_distance = msg.position.x
        self.target_height = msg.position.z
        self.start_x = self.current_x
        self.start_y = self.current_y
        
        rospy.loginfo(f"收到目标: 距离={self.target_distance:.2f}m, 高度={self.target_height:.2f}m")
        
        # 开始执行
        self.state = self.STATE_APPROACH
    
    def move_distance(self, distance, speed=0.1):
        """移动指定距离"""
        rospy.loginfo(f"移动 {distance:.2f}m")
        start_x = self.current_x
        start_y = self.current_y
        
        while not rospy.is_shutdown():
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            dist_moved = math.sqrt(dx*dx + dy*dy)
            
            if dist_moved >= abs(distance):
                self.stop()
                rospy.sleep(0.5)
                return True
            
            twist = Twist()
            twist.linear.x = speed if distance > 0 else -speed
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
    
    def stop(self):
        """停止底盘"""
        twist = Twist()
        self.cmd_pub.publish(twist)
    
    def control_arm(self, lift, gripper, duration=2.0):
        """控制机械臂，持续发送消息"""
        rospy.loginfo(f"控制机械臂: lift={lift:.2f}, gripper={gripper:.2f}")
        
        self.arm_msg.position = [lift, gripper]
        end_time = rospy.Time.now() + rospy.Duration(duration)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.arm_msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(self.arm_msg)
            rospy.sleep(0.1)
    
    def execute(self):
        """执行抓取流程"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            if self.state == self.STATE_IDLE:
                # 空闲状态，保持机械臂在准备位置
                self.arm_msg.header.stamp = rospy.Time.now()
                self.arm_pub.publish(self.arm_msg)
            
            elif self.state == self.STATE_APPROACH:
                rospy.loginfo("状态: 靠近目标")
                # 靠近目标，预留0.3m给机械臂
                move_dist = self.target_distance - 0.3
                if move_dist > 0:
                    self.move_distance(move_dist, 0.1)
                self.state = self.STATE_LOWER_ARM
            
            elif self.state == self.STATE_LOWER_ARM:
                rospy.loginfo("状态: 降下机械臂")
                # 降到目标高度
                lift_height = max(0.08, self.target_height - 0.05)
                self.control_arm(lift_height, 0.16, 3.0)  # 开爪
                self.state = self.STATE_GRAB
            
            elif self.state == self.STATE_GRAB:
                rospy.loginfo("状态: 抓取")
                # 闭合夹爪
                self.control_arm(self.target_height - 0.05, 0.03, 2.0)
                rospy.sleep(1)
                self.state = self.STATE_LIFT
            
            elif self.state == self.STATE_LIFT:
                rospy.loginfo("状态: 抬起")
                # 抬起物体
                self.control_arm(self.target_height + 0.1, 0.03, 3.0)
                rospy.sleep(1)
                self.state = self.STATE_RETREAT
            
            elif self.state == self.STATE_RETREAT:
                rospy.loginfo("状态: 后退")
                # 后退到起始位置
                self.move_distance(-0.3, 0.1)
                rospy.sleep(1)
                self.state = self.STATE_DONE
            
            elif self.state == self.STATE_DONE:
                rospy.loginfo("状态: 完成，等待2秒后复位")
                rospy.sleep(2)
                # 回到初始位置
                self.control_arm(0.08, 0.16, 3.0)
                rospy.sleep(1)
                self.state = self.STATE_IDLE
                rospy.loginfo("流程完成，等待新目标")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        fsm = WalleGrabFSM()
        fsm.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("FSM 被中断")
