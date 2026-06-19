#!/usr/bin/env python3
# real_fsm_node.py
# 真机专用分拣状态机（my_map.pgm / my_map.yaml）。
# 导航点 search_x/y、room_b_x/y、room_c_x/y 从 config/my_map_sorting.yaml 读取，
# 与仿真 stimulation/fsm_node.py 完全独立，改真机坐标不会影响仿真。
#
# 【话题接口说明】（明确区分“我用的”和“官方提供的”，避免冲突）
#   读取（订阅，只读，不会和官方抢）：
#     /odom                 nav_msgs/Odometry   官方底盘里程计(20Hz)，我用它算相对位移做闭环
#     /vision/box_camera    geometry_msgs/Pose  目标箱子在相机坐标/画面里的实时位置
#     /walle/command        std_msgs/String     命令行下达的指令
#     /walle/grasp_verify   (真机可选)          夹爪抓取成功检测
#   写入（发布）：
#     /cmd_vel              geometry_msgs/Twist  ★本节点是它的唯一发布者★
#     /wpb_home/mani_ctrl   sensor_msgs/JointState  官方机械臂控制接口(lift/gripper)
#   订阅（用于 mux 转发）：
#     /move_base/cmd_vel    move_base 的速度输出（launch 里已把 move_base 的 cmd_vel 重映射到此）
#
#   ★ /cmd_vel 冲突的根治办法（mux 思路）★
#     move_base 默认直接发 /cmd_vel，会和抓取时的手动控制抢，导致抖动/乱跑。
#     这里把 move_base 输出重映射到 /move_base/cmd_vel，本节点成为 /cmd_vel 唯一发布者：
#       - 导航模式 nav_mode=True ：把 /move_base/cmd_vel 转发到 /cmd_vel
#       - 抓取模式 nav_mode=False：忽略 move_base，发自己的速度
#     物理上保证任意时刻 /cmd_vel 只有一个来源，不可能冲突。
#
#   抓取对齐不用 map 坐标。只看摄像头当前画面里“命令指定颜色”的箱子：
#     - 横移：让目标箱子的相机横向 x 对到夹爪中心
#     - 旋转：让目标箱子的像素中心对到画面中心
#     - 前后：只用这个目标箱子的 depth 判断抓取阈值

import math
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion

try:
    from std_srvs.srv import Trigger
    _HAS_TRIGGER = True
except ImportError:
    _HAS_TRIGGER = False

_COLOR_CODES = {
    1: 'red',
    2: 'green',
    3: 'yellow',
    4: 'blue',
    5: 'brown',
    0: 'brown',
}


def _color_from_code(code_w):
    """Pose.orientation.w 存颜色码，用 round 避免 float 比较踩坑。"""
    return _COLOR_CODES.get(int(round(float(code_w))), 'brown')

class RealSortingFSM:
    def __init__(self):
        rospy.init_node('fsm_node')

        # ---------------- 房间坐标（my_map 系，务必在 my_map_sorting.yaml 标定）----------------
        self.room_b = (rospy.get_param('~room_b_x', -1.0),
                       rospy.get_param('~room_b_y', 2.0))
        self.room_c = (rospy.get_param('~room_c_x', 2.0),
                       rospy.get_param('~room_c_y', -2.0))

        # 桌前搜索位姿（先导航到这里，再执行相机抓取）
        self.search_x = rospy.get_param('~search_x', 0.5)
        self.search_y = rospy.get_param('~search_y', 1.0)
        self.search_yaw = rospy.get_param('~search_yaw', 0.0)

        # ---------------- 抓取参数（照官方，单位米）----------------
        # 抓取时物品停在底盘正前方的 x（机械臂伸出去够取的位置）
        # 太远会够不到→调小；机器人怼到桌子→调大。这是最主要的对距离旋钮。
        self.grab_object_x = rospy.get_param('~grab_object_x', 0.50)
        # 夹取时的升降高度（箱子中心约 0.83；有效范围 0.49~1.04）
        self.lift_grab_height = rospy.get_param('~lift_grab_height', 0.80)
        # 夹住后举起来搬运的高度（举高一点，搬运全程保持，不放下）
        self.lift_carry_height = rospy.get_param('~lift_carry_height', 0.95)
        # 放置时的降臂高度（房间地面放置，降低些再松手）
        self.place_height = rospy.get_param('~place_height', 0.55)
        self.gripper_open = rospy.get_param('~gripper_open', 0.16)
        self.gripper_close = rospy.get_param('~gripper_close', 0.032)
        self.arm_settle_time = rospy.get_param('~arm_settle_time', 3.5)
        # 相机闭环对位参数：只看命令指定颜色箱子的画面偏差和深度
        self.align_k_depth = rospy.get_param('~align_k_depth', 0.25)
        self.align_k_lateral = rospy.get_param('~align_k_lateral', 0.45)
        self.align_k_yaw = rospy.get_param('~align_k_yaw', 0.45)
        self.align_max_v = rospy.get_param('~align_max_v', 0.10)
        self.align_max_w = rospy.get_param('~align_max_w', 0.25)
        self.align_tol_depth = rospy.get_param('~align_tol_depth', 0.04)
        self.align_tol_lateral = rospy.get_param('~align_tol_lateral', 0.025)
        self.align_tol_yaw = rospy.get_param('~align_tol_yaw', 0.05)
        self.align_timeout = rospy.get_param('~align_timeout', 25.0)
        self.align_depth_slack = rospy.get_param('~align_depth_slack', 0.12)
        self.camera_stale_sec = rospy.get_param('~camera_stale_sec', 1.5)
        self.base_halt_sec = rospy.get_param('~base_halt_sec', 0.6)
        self.pre_grab_halt_sec = rospy.get_param('~pre_grab_halt_sec', 1.0)
        self.gripper_close_hold_sec = rospy.get_param('~gripper_close_hold_sec', 4.0)
        # 微调偏移（对不准时改这两个）
        self.grab_y_offset = rospy.get_param('~grab_y_offset', 0.0)
        self.grab_forward_offset = rospy.get_param('~grab_forward_offset', 0.0)
        # 相机里夹爪中心相对画面中心/相机光轴的偏移。
        # 若夹爪中心在画面偏左/偏右，用这两个参数校准，而不是改地图坐标。
        self.gripper_camera_x = rospy.get_param('~gripper_camera_x', 0.0)
        self.gripper_pixel_x = rospy.get_param('~gripper_pixel_x', 0.0)
        self.back_distance = rospy.get_param('~back_distance', 0.40)  # 抓到后后退距离

        # 闭环移动控制
        self.move_gain = rospy.get_param('~move_gain', 0.5)        # 比例增益
        self.move_max_v = rospy.get_param('~move_max_v', 0.30)     # 限速
        self.move_tol = rospy.get_param('~move_tol', 0.02)         # 到位容差
        self.move_timeout = rospy.get_param('~move_timeout', 20.0)

        # 抓取成功检测 + 重抓
        self.max_grab_attempts = rospy.get_param('~max_grab_attempts', 3)
        self.use_grasp_verify = rospy.get_param('~use_grasp_verify', False)
        self.grasp_verify_mode = rospy.get_param('~grasp_verify_mode', 'none')
        self.detect_timeout = rospy.get_param('~detect_timeout', 20.0)
        self.skip_navigation = rospy.get_param('~skip_navigation', True)
        self.return_to_search_after_task = rospy.get_param(
            '~return_to_search_after_task', False)

        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.set_initial_pose_flag = rospy.get_param('~set_initial_pose', False)

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ---------------- 发布/订阅 ----------------
        # fsm 是 /cmd_vel 的唯一发布者（mux）：
        #   导航模式 -> 转发 move_base 的速度；抓取模式 -> 发自己的速度。
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.arm_pub = rospy.Publisher('/wpb_home/mani_ctrl', JointState, queue_size=5)
        self.status_pub = rospy.Publisher('/task/status', String, queue_size=5)
        self.initpose_pub = rospy.Publisher('/initialpose',
                                            PoseWithCovarianceStamped, queue_size=2)
        # 导航模式开关：True 时把 move_base/cmd_vel 转发到 /cmd_vel，否则忽略它
        self.nav_mode = False
        rospy.Subscriber('/move_base/cmd_vel', Twist, self.nav_cmd_cb, queue_size=5)

        # /odom：官方底盘里程计（只读）。我用它算相对位移，自己掌控闭环，不碰官方 ctrl/pose_diff
        self.odom = None
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=10)

        # 视觉检测：颜色 -> detection，只保留最新一帧
        self.latest_by_color = {}
        self.latest_camera_by_color = {}
        self._vision_rx_logged = False
        rospy.Subscriber('/vision/box_camera', Pose, self.camera_detection_cb)
        rospy.Subscriber('/vision/box_detected', Pose, self.detection_cb)
        rospy.Subscriber('/walle/command', String, self.command_cb)

        self.grasp_verify_srv = None
        if self.use_grasp_verify and self.grasp_verify_mode == 'joint' and _HAS_TRIGGER:
            try:
                rospy.wait_for_service('/walle/grasp_verify', timeout=10.0)
                self.grasp_verify_srv = rospy.ServiceProxy('/walle/grasp_verify', Trigger)
                rospy.loginfo('已连接真机抓取检测服务 /walle/grasp_verify')
            except rospy.ROSException:
                rospy.logwarn('等待 /walle/grasp_verify 超时，抓取检测将跳过')

        self.busy = False
        self.state = 'idle'
        self.base_frozen = False

        # ---------------- 导航（skip_navigation 时不连接 move_base）----------------
        self.move_base = None
        if not self.skip_navigation:
            self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo('等待 move_base 服务器...')
            self.move_base.wait_for_server()
            rospy.loginfo('已连接 move_base')
        else:
            rospy.logwarn('skip_navigation=true：不启动 move_base 连接，纯视觉抓取模式')

        if self.set_initial_pose_flag:
            self.publish_initial_pose()

        rospy.loginfo('=' * 50)
        rospy.loginfo('真机分拣 FSM 就绪（地图: my_map）')
        rospy.loginfo('导航点: search=(%.2f, %.2f, yaw=%.2f)',
                      self.search_x, self.search_y, self.search_yaw)
        rospy.loginfo('房间 B=(%.2f, %.2f)  房间 C=(%.2f, %.2f)',
                      self.room_b[0], self.room_b[1],
                      self.room_c[0], self.room_c[1])
        if self.skip_navigation:
            rospy.logwarn('skip_navigation=true：不发 move_base 目标，在当前位置直接抓取')
        else:
            rospy.logwarn('每次启动必须 RViz 2D Pose Estimate；定位飘了会乱走')
        rospy.logwarn('请确认以上坐标已在 config/my_map_sorting.yaml 按 RViz 标定')
        rospy.loginfo("指令示例: rostopic pub -1 /walle/command std_msgs/String \"red B\"")
        rospy.loginfo('  颜色: red/green/yellow，房间: B 或 C')
        rospy.loginfo('=' * 50)

    # ========================================================
    # 回调
    # ========================================================
    def nav_cmd_cb(self, msg):
        """move_base 的速度只在导航模式下转发到 /cmd_vel；抓取模式下一律忽略。
        这样 /cmd_vel 永远只有 fsm 一个发布者，不会和抓取控制打架。"""
        if self.nav_mode and not self.base_frozen:
            self.cmd_pub.publish(msg)

    def odom_cb(self, msg):
        self.odom = msg

    def odom_pose(self):
        """从 /odom 取当前位姿 (x, y, yaw)。取不到返回 None。"""
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return p.x, p.y, yaw

    def detection_cb(self, msg):
        color = _color_from_code(msg.orientation.w)
        self.latest_by_color[color] = (msg.position.x, msg.position.y,
                                       msg.position.z, rospy.Time.now())

    def camera_detection_cb(self, msg):
        """与 stimulation/fsm_node 一致：直接采用视觉节点已选好的最近目标。"""
        color = _color_from_code(msg.orientation.w)
        if not self._vision_rx_logged:
            self._vision_rx_logged = True
            rospy.loginfo('FSM 已收到 /vision/box_camera（%s depth=%.2f）',
                          color, msg.position.z)
        self.latest_camera_by_color[color] = {
            'cam_x': msg.position.x,
            'cam_y': msg.position.y,
            'depth': msg.position.z,
            'pixel_x': msg.orientation.x,
            'pixel_y': msg.orientation.y,
            'area': msg.orientation.z,
            'stamp': rospy.Time.now()
        }

    def command_cb(self, msg):
        if self.busy:
            rospy.logwarn('正在执行任务，忽略新指令: %s', msg.data)
            return
        parts = msg.data.strip().split()
        color = None
        room = None
        for p in parts:
            pu = p.upper()
            if pu in ('B', 'C'):
                room = pu
            else:
                color = p.lower()
        if room is None:
            rospy.logwarn("指令无效: '%s'，需包含房间 B 或 C，如 'red B'", msg.data)
            return

        rospy.loginfo('收到指令: 颜色=%s 房间=%s', color or '(任意)', room)
        self.busy = True
        try:
            self.execute_task(color, room)
        except Exception as e:
            import traceback
            rospy.logerr('任务执行异常: %s\n%s', e, traceback.format_exc())
        finally:
            self.base_frozen = False
            self.halt_base(self.base_halt_sec)
            self.busy = False
            self.set_state('idle')

    # ========================================================
    # 主流程
    # ========================================================
    def execute_task(self, color, room):
        self.cancel_nav()
        # 1. 导航到桌前（skip_navigation 时跳过）
        if self.skip_navigation:
            rospy.logwarn('跳过导航，在当前位置执行抓取（请手动把车停在桌前）')
            self.set_state('at_table')
        else:
            self.set_state('goto_table')
            if not self.nav_to(self.search_x, self.search_y, self.search_yaw):
                rospy.logwarn('导航到桌前失败，放弃任务（检查 2D Pose Estimate 与 search 坐标）')
                return
        rospy.sleep(1.0)  # 等底盘稳定，pose 不抖

        # 2. 抓取。真机现场默认不自动重导航重试，避免失败后反复转圈/顶桌。
        grabbed = False
        for attempt in range(1, self.max_grab_attempts + 1):
            rospy.loginfo('======== 第 %d 次抓取 ========', attempt)
            if self.grab_once(color):
                grabbed = True
                break
            rospy.logwarn('第 %d 次抓取失败，停止任务，不再自动回桌前重试', attempt)
            break

        if not grabbed:
            rospy.logwarn('未抓到箱子，放弃任务并复位机械臂')
            self.reset_arm()
            return

        # 3. 送到目标房间
        if self.skip_navigation:
            rospy.logwarn(
                'skip_navigation=true，跳过送房间；抓取完成，保持举升姿态（不复位机械臂）')
            return

        self.set_state('to_room')
        rx, ry = self.room_b if room == 'B' else self.room_c
        if not self.nav_to(rx, ry, 0.0):
            rospy.logwarn('导航到房间 %s 失败，仍尝试放下', room)

        # 4. 放下
        self.set_state('placing')
        self.place_sequence()

        # 5. 现场调试默认不自动返航，避免放置后继续转圈/重复导航。
        if self.return_to_search_after_task:
            self.set_state('returning')
            self.nav_to(self.search_x, self.search_y, self.search_yaw)
        rospy.loginfo('任务完成：%s 箱子已送到房间 %s', color or '一个', room)

    def grab_once(self, color):
        """单次抓取：抬臂张爪 → 左右对齐 → 前后靠近 → 合夹爪 → 举起 → 后退。"""
        self.cancel_nav()
        self.base_frozen = True
        self.halt_base(self.base_halt_sec)

        # 1. 先确认摄像头能看到命令指定颜色的目标箱子
        self.set_state('scanning')
        if self.wait_see_camera_box(color) is None:
            rospy.logwarn('未检测到目标箱子')
            return False

        # 2. 先抬臂 + 张开夹爪（底盘不动）
        self.set_state('hand_up')
        rospy.loginfo('抬臂到 %.2fm 并张开夹爪', self.lift_grab_height)
        self.hold_arm(self.lift_grab_height, self.gripper_open, self.arm_settle_time)

        # 3a. 左右对齐：只横移 + 转向，不前进
        self.base_frozen = False
        self.set_state('align_lr')
        if not self.align_lateral_to_camera_box(color):
            self._log_align_fail(color, '左右对齐')
            return False

        # 3b. 前后靠近：只前进/后退调深度，不再横移打转
        self.set_state('align_fwd')
        if not self.align_depth_to_camera_box(color):
            self._log_align_fail(color, '前后靠近')
            return False

        # 4. 对位完成，停车锁定，合夹爪
        self.base_frozen = True
        self.halt_base(self.pre_grab_halt_sec)
        self.set_state('grab')
        rospy.logwarn(
            '===== 开始抓取：闭合夹爪 gripper %.3f -> %.3f =====',
            self.gripper_open, self.gripper_close)
        self.close_gripper(self.lift_grab_height)

        # 5. 举起箱子
        rospy.loginfo('夹爪已闭合，举起箱子到搬运高度 %.2fm', self.lift_carry_height)
        self.hold_arm(self.lift_carry_height, self.gripper_close, 2.5)

        # 6. 后退离开桌子
        self.base_frozen = False
        self.set_state('backward')
        self.move_rel(-self.back_distance, 0.0)

        # 7. 抓取成功检测
        ok = self.verify_grasp()
        if ok:
            rospy.loginfo('抓取成功，举着箱子搬运')
        else:
            rospy.logwarn('抓取检测：手上没有箱子，放下空爪重试')
            self.reset_arm()
        return ok

    # ========================================================
    # 视觉 + 闭环对位（与 stimulation/fsm_node.py 一致，仅用 /vision/box_camera）
    # ========================================================
    def wait_see_camera_box(self, color):
        """等到摄像头看见目标颜色箱子。"""
        start = rospy.Time.now()
        last_log = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            box = self.camera_box(color)
            if box is not None:
                rospy.loginfo(
                    '看到目标箱子 camera=(x %.3f, depth %.3f, pixel %.3f)',
                    box['cam_x'], box['depth'], box['pixel_x'])
                return box
            if (rospy.Time.now() - last_log).to_sec() > 2.0:
                now = rospy.Time.now()
                stale = self.camera_stale_sec
                status = {}
                for c, b in self.latest_camera_by_color.items():
                    age = (now - b['stamp']).to_sec()
                    status[c] = 'depth=%.2f age=%.1fs%s' % (
                        b['depth'], age, ' STALE' if age > stale else '')
                rospy.loginfo(
                    '等待 %s 中… FSM 缓存: %s',
                    color or '(任意)', status or '(空)')
                last_log = rospy.Time.now()
            if (rospy.Time.now() - start).to_sec() > self.detect_timeout:
                rospy.logwarn(
                    '等待 %s 超时，相机当前颜色: %s',
                    color, list(self.latest_camera_by_color.keys()))
                return None
            rate.sleep()
        return None

    def camera_box(self, color):
        """取指定颜色的最新相机检测。color 为空时取任意颜色中面积最大的。"""
        now = rospy.Time.now()
        stale = self.camera_stale_sec
        if color is not None:
            box = self.latest_camera_by_color.get(color)
            if box is None or (now - box['stamp']).to_sec() > stale:
                return None
            return box

        candidates = [
            box for box in self.latest_camera_by_color.values()
            if (now - box['stamp']).to_sec() <= stale
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda b: b['depth'])

    def _align_targets(self):
        return (
            self.grab_object_x + self.grab_forward_offset,
            self.gripper_camera_x + self.grab_y_offset,
            self.gripper_pixel_x,
        )

    def _camera_box_cached(self, color, last_box):
        box = self.camera_box(color)
        if box is None and last_box is not None:
            if (rospy.Time.now() - last_box['stamp']).to_sec() <= self.camera_stale_sec:
                box = last_box
        return box

    def _log_align_fail(self, color, phase_name):
        self.base_frozen = True
        self.halt_base(self.base_halt_sec)
        box = self.camera_box(color)
        target_depth, target_x, target_pixel_x = self._align_targets()
        if box is not None:
            e_depth = box['depth'] - target_depth
            e_lateral = box['cam_x'] - target_x
            e_yaw = box['pixel_x'] - target_pixel_x
            rospy.logwarn(
                '%s失败（仍看见 %s）: e_depth=%.3f e_x=%.3f e_yaw=%.3f (目标 depth≈%.2f)',
                phase_name, color, e_depth, e_lateral, e_yaw, target_depth)
        else:
            rospy.logwarn(
                '%s失败：丢失 %s，当前相机里有 %s',
                phase_name, color, list(self.latest_camera_by_color.keys()))

    def align_lateral_to_camera_box(self, color):
        """阶段 1：左右对齐。只发横移 linear.y + 转向 angular.z，不发前进。"""
        target_depth, target_x, target_pixel_x = self._align_targets()
        rospy.loginfo(
            '开始左右对齐: 目标 depth=%.2f cam_x=%.3f pixel=%.3f',
            target_depth, target_x, target_pixel_x)
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        last_log = rospy.Time.now()
        last_box = None

        while not rospy.is_shutdown():
            box = self._camera_box_cached(color, last_box)
            if box is not None:
                last_box = box
            if box is None:
                self.stop_base()
                if (rospy.Time.now() - start).to_sec() > self.align_timeout:
                    rospy.logwarn('左右对齐超时：看不到箱子')
                    return False
                rate.sleep()
                continue

            cam_x = box['cam_x']
            pixel_x = box['pixel_x']
            e_lateral = cam_x - target_x
            e_yaw = pixel_x - target_pixel_x

            if (rospy.Time.now() - last_log).to_sec() > 1.0:
                rospy.loginfo(
                    '[左右对齐] cam_x=%.3f e_x=%.3f pixel=%.3f e_yaw=%.3f',
                    cam_x, e_lateral, pixel_x, e_yaw)
                last_log = rospy.Time.now()

            if (abs(e_lateral) < self.align_tol_lateral and
                    abs(e_yaw) < self.align_tol_yaw):
                self.stop_base()
                rospy.loginfo(
                    '左右对齐完成: cam_x=%.3f pixel=%.3f',
                    cam_x, pixel_x)
                return True

            if (rospy.Time.now() - start).to_sec() > self.align_timeout:
                self.stop_base()
                rospy.logwarn(
                    '左右对齐超时: e_x=%.3f e_yaw=%.3f',
                    e_lateral, e_yaw)
                return False

            tw = Twist()
            tw.linear.y = self.clamp(-self.align_k_lateral * e_lateral,
                                     self.align_max_v)
            tw.angular.z = self.clamp(-self.align_k_yaw * e_yaw,
                                      self.align_max_w)
            self.publish_cmd(tw)
            rate.sleep()
        return False

    def align_depth_to_camera_box(self, color):
        """阶段 2：前后靠近。只发前进 linear.x，直到深度到位。"""
        target_depth, target_x, target_pixel_x = self._align_targets()
        rospy.loginfo(
            '开始前后靠近: 目标 depth=%.2f (容差 %.2f)',
            target_depth, self.align_tol_depth)
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        last_log = rospy.Time.now()
        last_box = None

        while not rospy.is_shutdown():
            box = self._camera_box_cached(color, last_box)
            if box is not None:
                last_box = box
            if box is None:
                self.stop_base()
                if (rospy.Time.now() - start).to_sec() > self.align_timeout:
                    rospy.logwarn('前后靠近超时：看不到箱子')
                    return False
                rate.sleep()
                continue

            depth = box['depth']
            e_depth = depth - target_depth
            depth_ok = abs(e_depth) < self.align_tol_depth
            within_reach = (depth <= target_depth + self.align_depth_slack and
                            depth >= 0.25)

            if (rospy.Time.now() - last_log).to_sec() > 1.0:
                rospy.loginfo(
                    '[前后靠近] depth=%.3f e_depth=%.3f (目标 %.2f)',
                    depth, e_depth, target_depth)
                last_log = rospy.Time.now()

            if depth_ok or within_reach:
                self._finish_align(depth, box['cam_x'], box['pixel_x'])
                return True

            if (rospy.Time.now() - start).to_sec() > self.align_timeout:
                self.stop_base()
                rospy.logwarn(
                    '前后靠近超时: depth=%.3f e_depth=%.3f',
                    depth, e_depth)
                return False

            tw = Twist()
            tw.linear.x = self.clamp(self.align_k_depth * e_depth, self.align_max_v)
            self.publish_cmd(tw)
            rate.sleep()
        return False

    def align_to_camera_box(self, color):
        """兼容旧调用：左右对齐 → 前后靠近。"""
        return (self.align_lateral_to_camera_box(color) and
                self.align_depth_to_camera_box(color))

    def _finish_align(self, depth, cam_x, pixel_x):
        rospy.loginfo(
            '相机对位完成: depth=%.3f cam_x=%.3f pixel=%.3f',
            depth, cam_x, pixel_x)
        self.halt_base(self.base_halt_sec)

    def pick_box(self, color):
        now = rospy.Time.now()
        candidates = []
        for c, (x, y, z, stamp) in self.latest_by_color.items():
            if (now - stamp).to_sec() > 3.0:
                continue
            if color is not None and c != color:
                continue
            candidates.append((x, y, z))
        if not candidates:
            return None
        rp = self.robot_pose()
        if rp is None:
            return candidates[0]
        rx, ry, _ = rp
        candidates.sort(key=lambda b: (b[0] - rx) ** 2 + (b[1] - ry) ** 2)
        return candidates[0]

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', self.base_frame,
                                                rospy.Time(0), rospy.Duration(0.5))
            tr = t.transform.translation
            rot = t.transform.rotation
            yaw = math.atan2(2.0 * (rot.w * rot.z + rot.x * rot.y),
                             1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z))
            return tr.x, tr.y, yaw
        except Exception:
            return None

    # ========================================================
    # 抓取成功检测（真机：夹爪 joint 或默认信任闭合）
    # ========================================================
    def verify_grasp(self):
        if not self.use_grasp_verify or self.grasp_verify_mode == 'none':
            return True
        if self.grasp_verify_srv is None:
            rospy.logwarn('未连接真机抓取检测，跳过验证')
            return True
        try:
            resp = self.grasp_verify_srv()
            return resp.success
        except rospy.ServiceException as exc:
            rospy.logwarn('抓取检测服务调用失败: %s', exc)
            return True

    # ========================================================
    # 运动：导航(move_base) + /odom 闭环平移
    # ========================================================
    def nav_to(self, x, y, yaw):
        if self.move_base is None:
            rospy.logwarn('move_base 未启用，跳过导航')
            return False
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        rospy.loginfo('导航到 (%.2f, %.2f)', x, y)
        self.nav_mode = True          # 开启转发：导航期间 /cmd_vel 来自 move_base
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result(rospy.Duration(120.0))
        state = self.move_base.get_state()
        # 导航结束：关闭转发 + 取消目标 + 停车，之后由抓取控制独占 /cmd_vel
        self.cancel_nav()
        return state == actionlib.GoalStatus.SUCCEEDED

    def cancel_nav(self):
        """关闭导航转发并取消 move_base 目标，确保之后只有抓取控制发 /cmd_vel。"""
        self.nav_mode = False
        if self.move_base is not None:
            self.move_base.cancel_all_goals()
        rospy.sleep(0.3)
        self.stop_base()

    def move_rel(self, tx, ty):
        """相对当前位置平移 (tx 前进, ty 左移) 米。
        用只读 /odom 记起点算位移做闭环，纯平移不旋转，完全自掌控，不碰官方接口。"""
        # 等 /odom 就绪
        t_wait = rospy.Time.now()
        while self.odom_pose() is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t_wait).to_sec() > 3.0:
                rospy.logwarn('move_rel: 收不到 /odom，放弃移动')
                return False
            rospy.sleep(0.05)

        sx, sy, syaw = self.odom_pose()
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            cur = self.odom_pose()
            if cur is None:
                self.stop_base()
                rate.sleep()
                continue
            # 当前相对起点的位移，投影到“起点朝向”得到前进/左移分量
            dx = cur[0] - sx
            dy = cur[1] - sy
            fwd = dx * math.cos(syaw) + dy * math.sin(syaw)
            left = -dx * math.sin(syaw) + dy * math.cos(syaw)
            ex = tx - fwd
            ey = ty - left
            if abs(ex) < self.move_tol and abs(ey) < self.move_tol:
                self.stop_base()
                rospy.loginfo('移动完成: 目标(%.3f,%.3f) 实到(%.3f,%.3f)',
                              tx, ty, fwd, left)
                return True
            if (rospy.Time.now() - start).to_sec() > self.move_timeout:
                self.stop_base()
                rospy.logwarn('移动超时: 目标(%.3f,%.3f) 实到(%.3f,%.3f)',
                              tx, ty, fwd, left)
                return False
            tw = Twist()
            tw.linear.x = self.clamp(self.move_gain * ex, self.move_max_v)
            tw.linear.y = self.clamp(self.move_gain * ey, self.move_max_v)
            self.publish_cmd(tw)
            rate.sleep()
        return False

    def publish_cmd(self, tw):
        """底盘速度出口。base_frozen 时强制零速度，防止机械臂动作期间车还在动。"""
        if self.base_frozen:
            self.cmd_pub.publish(Twist())
            return
        self.cmd_pub.publish(tw)

    def halt_base(self, hold_sec=None):
        """连续发零速度并取消导航，抵消驱动器保持上一帧速度指令的惯性。"""
        if hold_sec is None:
            hold_sec = self.base_halt_sec
        self.nav_mode = False
        if self.move_base is not None:
            try:
                self.move_base.cancel_all_goals()
            except Exception:
                pass
        zero = Twist()
        rate = rospy.Rate(20)
        end = rospy.Time.now() + rospy.Duration(hold_sec)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_pub.publish(zero)
            rate.sleep()

    def wait_with_base_halt(self, duration):
        """等待机械臂动作时持续发零速度，避免抬臂/夹取期间底盘偷偷动。"""
        rate = rospy.Rate(20)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_pub.publish(Twist())
            rate.sleep()

    def stop_base(self):
        self.halt_base(0.15)

    # ========================================================
    # 机械臂
    # ========================================================
    def arm_cmd(self, lift, gripper, hold_sec=0.5):
        self.hold_arm(lift, gripper, hold_sec)

    def hold_arm(self, lift, gripper, duration):
        """持续发布 mani_ctrl（30Hz），与官方 wpb_home_grab_server 一致。"""
        msg = JointState()
        msg.name = ['lift', 'gripper']
        msg.position = [lift, gripper]
        msg.velocity = [0.5, 5.0]
        rate = rospy.Rate(30)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            if self.base_frozen:
                self.cmd_pub.publish(Twist())
            msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(msg)
            rate.sleep()

    def close_gripper(self, lift_height):
        """抓取：长时间、高频率重复发闭合指令，确保夹爪真合上。"""
        msg = JointState()
        msg.name = ['lift', 'gripper']
        msg.position = [lift_height, self.gripper_close]
        msg.velocity = [0.5, 5.0]
        total = max(60, int(self.gripper_close_hold_sec * 30))
        rate = rospy.Rate(30)
        for i in range(total):
            if self.base_frozen:
                self.cmd_pub.publish(Twist())
            msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(msg)
            if i == 0 or (i + 1) % 30 == 0:
                rospy.loginfo(
                    '夹爪闭合中 %d/%d (lift=%.2f gripper=%.3f)',
                    i + 1, total, lift_height, self.gripper_close)
            rate.sleep()

    def reset_arm(self):
        """机械臂复位：降下 + 张开（底盘锁定）。"""
        was_frozen = self.base_frozen
        self.base_frozen = True
        self.halt_base(self.base_halt_sec)
        self.arm_cmd(0.0, self.gripper_open, hold_sec=1.0)
        self.wait_with_base_halt(1.0)
        if not was_frozen:
            self.base_frozen = False

    def place_sequence(self):
        """放置：从搬运高度降到放置高度 -> 张爪释放 -> 后退 -> 复位。"""
        rospy.loginfo('开始放置...')
        self.base_frozen = True
        self.halt_base(self.base_halt_sec)
        # 从搬运高度降到放置高度（保持夹紧）
        self.arm_cmd(self.place_height, self.gripper_close)
        self.wait_with_base_halt(2.5)
        # 松爪释放
        self.arm_cmd(self.place_height, self.gripper_open)
        self.wait_with_base_halt(1.5)
        # 后退后复位
        self.base_frozen = False
        self.move_rel(-self.back_distance, 0.0)
        self.reset_arm()
        self.base_frozen = False
        rospy.loginfo('放置完成')

    # ========================================================
    @staticmethod
    def clamp(v, lim):
        return max(-lim, min(lim, v))

    def set_state(self, s):
        self.state = s
        self.status_pub.publish(String(data=s))
        rospy.loginfo('[状态] %s', s)

    def publish_initial_pose(self):
        """仿真调试可选：自动发布 AMCL 初始位姿。真机默认关闭，用 RViz 2D Pose Estimate。"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        q = quaternion_from_euler(0, 0, 0)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        for _ in range(3):
            msg.header.stamp = rospy.Time.now()
            self.initpose_pub.publish(msg)
            rospy.sleep(0.3)
        rospy.loginfo('已发布 AMCL 初始位姿 (0,0,0) [仿真调试]')

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        RealSortingFSM().run()
    except rospy.ROSInterruptException:
        rospy.loginfo('真机 FSM 被中断')
