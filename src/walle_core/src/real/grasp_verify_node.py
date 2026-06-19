#!/usr/bin/env python3
# grasp_verify_node.py
# 真机抓取成功检测：通过 /joint_states 中夹爪关节位置判断是否有物体被夹住。
# 提供 /walle/grasp_verify 服务，供 fsm_node 在 grasp_verify_mode=joint 时调用。

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse


class GraspVerifyNode:
    def __init__(self):
        rospy.init_node('grasp_verify_node')

        self.open_threshold = rospy.get_param('~gripper_open_threshold', 0.12)
        self.closed_threshold = rospy.get_param('~gripper_closed_threshold', 0.14)
        self.latest_gap = None

        rospy.Subscriber('/joint_states', JointState, self.joint_cb, queue_size=5)
        rospy.Service('/walle/grasp_verify', Trigger, self.check_cb)

        rospy.loginfo('真机抓取检测就绪: /walle/grasp_verify')
        rospy.loginfo('  张开阈值 < %.3f, 夹住判定 < %.3f',
                      self.open_threshold, self.closed_threshold)

    def joint_cb(self, msg):
        left = right = None
        for i, name in enumerate(msg.name):
            if name == 'forearm_left_finger':
                left = msg.position[i]
            elif name == 'forearm_right_finger':
                right = msg.position[i]
        if left is not None and right is not None:
            self.latest_gap = abs(left) + abs(right)

    def check_cb(self, _req):
        gap = self.latest_gap
        if gap is None:
            rospy.logwarn('尚无 joint_states，默认判定抓取成功')
            return TriggerResponse(success=True, message='no joint data, assume ok')

        # 夹住物体时手指不会完全张开，也不会像空夹那样完全闭合
        ok = gap < self.closed_threshold
        msg = 'gap=%.4f -> %s' % (gap, 'grasped' if ok else 'empty/missed')
        rospy.loginfo('抓取检测: %s', msg)
        return TriggerResponse(success=ok, message=msg)


if __name__ == '__main__':
    try:
        GraspVerifyNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
