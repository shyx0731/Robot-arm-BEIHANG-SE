#!/usr/bin/env python3
# nav_node.py
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point
from std_srvs.srv import Trigger, TriggerResponse

class NavNode:
    def __init__(self):
        rospy.init_node('nav_node')
        
        # move_base客户端
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        self.move_base.wait_for_server(rospy.Duration(5.0))
        
        # 服务
        rospy.Service('/nav/go_to', Trigger, self.go_to_position)
        
        rospy.loginfo("导航节点已启动")
    
    def go_to_position(self, req):
        """导航到指定位置"""
        # 在实际中，这个服务应该接收目标位置参数
        # 这里简化为固定位置
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置目标位置
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo(f"导航到位置: ({1.0}, {0.0})")
        self.move_base.send_goal(goal)
        
        # 等待结果
        self.move_base.wait_for_result()
        result = self.move_base.get_state()
        
        if result == actionlib.GoalStatus.SUCCEEDED:
            return TriggerResponse(success=True, message="导航成功")
        else:
            return TriggerResponse(success=False, message="导航失败")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    nn = NavNode()
    nn.run()
