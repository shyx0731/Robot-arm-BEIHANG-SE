#!/usr/bin/env python3
# real_zone_manager.py
# 真机专用区域管理（my_map），与 stimulation/zone_manager.py 独立。
import rospy
import json
import os
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse


class ZoneManager:
    def __init__(self):
        rospy.init_node('zone_manager')

        self.zone_a = {
            'min': Point(
                x=rospy.get_param('~zone_a_min_x', 0.0),
                y=rospy.get_param('~zone_a_min_y', 0.0),
                z=0),
            'max': Point(
                x=rospy.get_param('~zone_a_max_x', 1.0),
                y=rospy.get_param('~zone_a_max_y', 1.0),
                z=0),
            'height': rospy.get_param('~table_height', 0.75),
            'description': '源区域 (各种箱子)'
        }

        self.zone_b = {
            'min': Point(
                x=rospy.get_param('~zone_b_min_x', -2.0),
                y=rospy.get_param('~zone_b_min_y', 1.0),
                z=0),
            'max': Point(
                x=rospy.get_param('~zone_b_max_x', 0.0),
                y=rospy.get_param('~zone_b_max_y', 3.0),
                z=0),
            'height': rospy.get_param('~table_height', 0.75),
            'description': '本色箱子区'
        }

        self.zone_c = {
            'min': Point(
                x=rospy.get_param('~zone_c_min_x', 1.0),
                y=rospy.get_param('~zone_c_min_y', -3.0),
                z=0),
            'max': Point(
                x=rospy.get_param('~zone_c_max_x', 3.0),
                y=rospy.get_param('~zone_c_max_y', -1.0),
                z=0),
            'height': rospy.get_param('~table_height', 0.75),
            'description': '彩色箱子区'
        }

        self.home_position = Pose()
        self.home_position.position.x = rospy.get_param('~home_x', 0.0)
        self.home_position.position.y = rospy.get_param('~home_y', 0.0)
        self.home_position.orientation.w = 1.0

        self.dedup_threshold = rospy.get_param('~dedup_threshold', 0.15)
        self.boxes = []

        self.zone_pub = rospy.Publisher('/zones/info', String, queue_size=10)
        self.box_pub = rospy.Publisher('/boxes/info', String, queue_size=10)

        # 不再记录箱子：抓取决策已移到 fsm_node（视觉实时对位），
        # 这里只保留区域信息发布，避免重复记录/刷屏。
        # rospy.Subscriber('/vision/box_detected', Pose, self.box_detected_cb)

        rospy.Service('/zones/save', Empty, self.save_zones)
        rospy.Service('/zones/load', Empty, self.load_zones)
        rospy.Service('/zones/reset', Empty, self.reset_zones)

        self.config_dir = os.path.expanduser('~/.ros/walle_zones')
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)

        rospy.loginfo('真机区域管理器已启动 (my_map，参数来自 my_map_sorting.yaml)')
        self.publish_zones()
        rospy.Timer(rospy.Duration(2.0), self.timer_publish_callback)

    def timer_publish_callback(self, event):
        try:
            self.publish_zones()
        except Exception as e:
            rospy.logerr('定时发布区域信息失败: %s', e)

    def _is_duplicate_box(self, x, y, z):
        for box in self.boxes:
            if box['status'] in ('detected', 'grabbed'):
                pos = box['position']
                dist = ((pos['x'] - x) ** 2 + (pos['y'] - y) ** 2 +
                        (pos['z'] - z) ** 2) ** 0.5
                if dist < self.dedup_threshold:
                    return True
        return False

    def box_detected_cb(self, msg):
        try:
            rospy.loginfo('[ZONE] 收到箱子坐标: x=%.2f, y=%.2f, z=%.2f',
                          msg.position.x, msg.position.y, msg.position.z)

            if self._is_duplicate_box(msg.position.x, msg.position.y, msg.position.z):
                rospy.logdebug('[ZONE] 跳过重复箱子检测')
                return

            box_id = len(self.boxes) + 1
            color_code = msg.orientation.w
            color_map = {
                1.0: 'red',
                2.0: 'green',
                3.0: 'yellow',
                4.0: 'blue',
                5.0: 'brown',
                0.0: 'brown'
            }
            color = color_map.get(color_code, 'brown')
            target_zone = 'B' if color == 'brown' else 'C'

            new_box = {
                'id': box_id,
                'color': color,
                'position': {
                    'x': msg.position.x,
                    'y': msg.position.y,
                    'z': msg.position.z
                },
                'status': 'detected',
                'target_zone': target_zone,
                'target_position': self.get_next_position_in_zone(target_zone)
            }

            self.boxes.append(new_box)
            rospy.loginfo('[ZONE] 已添加箱子 %d (%s)，总数: %d',
                          new_box['id'], color, len(self.boxes))
            self.publish_boxes()

        except Exception as e:
            rospy.logerr('[ZONE] box_detected_cb 错误: %s', e)
            import traceback
            rospy.logerr(traceback.format_exc())

    def get_next_position_in_zone(self, zone_letter):
        if zone_letter == 'B':
            zone = self.zone_b
        elif zone_letter == 'C':
            zone = self.zone_c
        else:
            return None

        placed_count = len([
            b for b in self.boxes
            if b.get('status') == 'placed' and b.get('target_zone') == zone_letter
        ])

        rows, cols = 3, 3
        row = placed_count // cols
        col = placed_count % cols

        pos = Point()
        pos.x = zone['min'].x + (zone['max'].x - zone['min'].x) * (col + 0.5) / cols
        pos.y = zone['min'].y + (zone['max'].y - zone['min'].y) * (row + 0.5) / rows
        pos.z = zone['height']

        return {'x': pos.x, 'y': pos.y, 'z': pos.z}

    def box_grabbed_cb(self, msg):
        box_id = int(msg.data)
        for box in self.boxes:
            if box['id'] == box_id:
                box['status'] = 'grabbed'
                rospy.loginfo('箱子 %d 被抓取', box_id)
                break
        self.publish_boxes()

    def box_placed_cb(self, msg):
        box_id = int(msg.data)
        for box in self.boxes:
            if box['id'] == box_id:
                box['status'] = 'placed'
                rospy.loginfo('箱子 %d 被放置到区域 %s', box_id, box['target_zone'])
                break
        self.publish_boxes()

    def publish_zones(self):
        zones_info = {
            'zone_a': {
                'min': {'x': self.zone_a['min'].x, 'y': self.zone_a['min'].y,
                        'z': self.zone_a['min'].z},
                'max': {'x': self.zone_a['max'].x, 'y': self.zone_a['max'].y,
                        'z': self.zone_a['max'].z},
                'height': self.zone_a['height'],
                'description': self.zone_a['description']
            },
            'zone_b': {
                'min': {'x': self.zone_b['min'].x, 'y': self.zone_b['min'].y,
                        'z': self.zone_b['min'].z},
                'max': {'x': self.zone_b['max'].x, 'y': self.zone_b['max'].y,
                        'z': self.zone_b['max'].z},
                'height': self.zone_b['height'],
                'description': self.zone_b['description']
            },
            'zone_c': {
                'min': {'x': self.zone_c['min'].x, 'y': self.zone_c['min'].y,
                        'z': self.zone_c['min'].z},
                'max': {'x': self.zone_c['max'].x, 'y': self.zone_c['max'].y,
                        'z': self.zone_c['max'].z},
                'height': self.zone_c['height'],
                'description': self.zone_c['description']
            },
            'home': {
                'x': self.home_position.position.x,
                'y': self.home_position.position.y
            }
        }

        msg = String()
        msg.data = json.dumps(zones_info)
        self.zone_pub.publish(msg)

    def publish_boxes(self):
        try:
            if not self.boxes:
                return

            msg = String()
            msg.data = json.dumps(self.boxes, default=str)
            self.box_pub.publish(msg)
            rospy.loginfo('[ZONE] 发布 %d 个箱子到 /boxes/info', len(self.boxes))

        except Exception as e:
            rospy.logerr('[ZONE] 发布 /boxes/info 失败: %s', e)

    def save_zones(self, req):
        config = {
            'zone_a': self.zone_a,
            'zone_b': self.zone_b,
            'zone_c': self.zone_c,
            'home_position': {
                'x': self.home_position.position.x,
                'y': self.home_position.position.y
            }
        }

        with open(os.path.join(self.config_dir, 'zones.json'), 'w') as f:
            json.dump(config, f, default=lambda o: o.__dict__ if hasattr(o, '__dict__') else str(o))

        rospy.loginfo('区域配置已保存')
        return EmptyResponse()

    def load_zones(self, req):
        config_file = os.path.join(self.config_dir, 'zones.json')
        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = json.load(f)
                rospy.loginfo('加载区域配置: %s', config)
        return EmptyResponse()

    def reset_zones(self, req):
        self.boxes = []
        rospy.loginfo('区域已重置')
        return EmptyResponse()

    def get_next_task(self):
        for box in self.boxes:
            if box['status'] == 'detected':
                return box
        return None

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    zm = ZoneManager()
    zm.run()
