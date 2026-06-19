#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Twist

class NavNode:
    def __init__(self):
        rospy.init_node('walle_nav_node')
        self.target_sub = rospy.Subscriber("/walle/target_info", Pose, self.target_cb)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.loginfo("[驾驶员] 导航节点已上线，等待目标坐标...")

    def target_cb(self, msg):
        rospy.loginfo(f"[驾驶员] 收到目标！距离: {msg.position.x}m。准备发车！")
        # TODO: 这里写速度控制算法 (PID)，让底盘靠近目标
        
if __name__ == '__main__':
    n = NavNode()
    rospy.spin()
