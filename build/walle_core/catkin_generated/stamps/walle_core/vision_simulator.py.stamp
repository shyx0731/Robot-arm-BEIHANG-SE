#!/usr/bin/env python3
# vision_simulator.py
import rospy
import random
import json
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class VisionSimulator:
    def __init__(self):
        rospy.init_node('vision_simulator')
        
        # 模拟的箱子检测结果
        self.detected_boxes = []
        
        # 发布检测到的箱子
        self.box_pub = rospy.Publisher('/vision/box_detected', Pose, queue_size=10)
        
        # 服务
        rospy.Service('/vision/scan_zone_a', Trigger, self.scan_zone_a)
        
        rospy.Subscriber('/zones/info', String, self.zones_cb)
        
        self.zones = None
        
        rospy.loginfo("视觉模拟器已启动")
    
    def zones_cb(self, msg):
        """接收区域信息"""
        self.zones = json.loads(msg.data)
    
    def scan_zone_a(self, req):
        """扫描区域A，模拟检测箱子"""
        # 1. 健壮性检查：确保已收到区域信息，并且格式正确
        if self.zones is None:
            rospy.logerr("视觉服务错误：尚未收到任何区域信息 (/zones/info)。请确认zone_manager节点已启动。")
            return TriggerResponse(success=False, message="No zones information received yet.")
    
        # 检查 self.zones 是否为字典类型
        if not isinstance(self.zones, dict):
            rospy.logerr(f"视觉服务错误：区域信息格式异常。期望字典，实际得到: {type(self.zones)}")
            return TriggerResponse(success=False, message=f"Invalid zones data type: {type(self.zones)}")
    
        # 检查必需的 'zone_a' 键是否存在
        if 'zone_a' not in self.zones:
            rospy.logerr(f"视觉服务错误：区域信息中未找到 'zone_a'。现有键: {self.zones.keys()}")
            return TriggerResponse(success=False, message="Key 'zone_a' not found in zones info.")
    
        # 2. 现在可以安全地访问 zone_a
        zone_a = self.zones['zone_a']
    
        # 3. 可选：进一步检查 zone_a 的结构
        required_keys = ['min', 'max', 'height']
        for key in required_keys:
            if key not in zone_a:
                rospy.logerr(f"视觉服务错误：zone_a 中缺少必需键 '{key}'")
                return TriggerResponse(success=False, message=f"Key '{key}' missing in zone_a.")
    
    # --- 以下是原有的模拟检测逻辑（保持不变）---
        num_boxes = random.randint(3, 8)
        rospy.loginfo(f"模拟检测到 {num_boxes} 个箱子")
    
        for i in range(num_boxes):
            x = random.uniform(zone_a['min']['x'], zone_a['max']['x'])
            y = random.uniform(zone_a['min']['y'], zone_a['max']['y'])
            box_pose = Pose()
            box_pose.position.x = x
            box_pose.position.y = y
            box_pose.position.z = zone_a['height']
            rospy.sleep(0.1)
            self.box_pub.publish(box_pose)
    # --- 原有逻辑结束 ---
    
        return TriggerResponse(success=True, message=f"检测到 {num_boxes} 个箱子")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    vs = VisionSimulator()
    vs.run()
