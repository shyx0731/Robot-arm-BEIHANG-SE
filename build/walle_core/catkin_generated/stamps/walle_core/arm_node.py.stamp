#!/usr/bin/env python3
# arm_node.py
import rospy
import json
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class ArmNode:
    def __init__(self):
        rospy.init_node('arm_node')
        
        # 机械臂控制
        self.arm_pub = rospy.Publisher('/wpb_home/mani_ctrl', JointState, queue_size=5)
        
        # 消息模板
        self.arm_msg = JointState()
        self.arm_msg.name = ["lift", "gripper"]
        self.arm_msg.position = [0.08, 0.16]  # 准备位置
        self.arm_msg.velocity = [0.2, 5.0]
        
        # 服务
        rospy.Service('/arm/grab', Trigger, self.grab_box)
        rospy.Service('/arm/place', Trigger, self.place_box)
        rospy.Service('/arm/prepare', Trigger, self.prepare_arm)
        
        rospy.Subscriber('/task/box_position', Pose, self.box_position_cb)
        
        self.current_box_height = 0.75  # 默认桌子高度
        
        rospy.loginfo("机械臂节点已启动")
    
    def box_position_cb(self, msg):
        """接收箱子位置信息"""
        self.current_box_height = msg.position.z
    
    def prepare_arm(self, req):
        """准备机械臂（回到初始位置）"""
        rospy.loginfo("准备机械臂...")
        self.arm_msg.position = [0.08, 0.16]  # 降臂+开爪
        for _ in range(10):
            self.arm_msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(self.arm_msg)
            rospy.sleep(0.2)
        return TriggerResponse(success=True, message="机械臂准备就绪")
    
    def grab_box(self, req):
        """抓取箱子"""
        rospy.loginfo("执行抓取...")
        
        # 1. 降臂到箱子高度
        self.arm_msg.position = [self.current_box_height, 0.16]  # 开爪
        for _ in range(10):
            self.arm_msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(self.arm_msg)
            rospy.sleep(0.2)
        
        rospy.sleep(1)
        
        # 2. 闭合夹爪
        self.arm_msg.position = [self.current_box_height, 0.03]  # 闭爪
        for _ in range(10):
            self.arm_msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(self.arm_msg)
            rospy.sleep(0.2)
        
        rospy.sleep(1)
        
        return TriggerResponse(success=True, message="抓取完成")
    
    def place_box(self, req):
        """放置箱子"""
        rospy.loginfo("执行放置...")
        
        # 1. 抬升箱子
        lift_height = self.current_box_height + 0.2
        self.arm_msg.position = [lift_height, 0.03]  # 抬升
        for _ in range(10):
            self.arm_msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(self.arm_msg)
            rospy.sleep(0.2)
        
        rospy.sleep(1)
        
        # 2. 打开夹爪
        self.arm_msg.position = [self.current_box_height, 0.16]  # 开爪
        for _ in range(10):
            self.arm_msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(self.arm_msg)
            rospy.sleep(0.2)
        
        rospy.sleep(1)
        
        return TriggerResponse(success=True, message="放置完成")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    an = ArmNode()
    an.run()
