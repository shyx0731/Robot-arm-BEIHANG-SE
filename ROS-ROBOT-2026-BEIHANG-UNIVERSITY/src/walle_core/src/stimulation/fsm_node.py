#!/usr/bin/env python3
# fsm_node.py
# 指令驱动的分拣状态机。
#
# 【话题接口说明】（明确区分“我用的”和“官方提供的”，避免冲突）
#   读取（订阅，只读，不会和官方抢）：
#     /odom                 nav_msgs/Odometry   官方底盘里程计(20Hz)，我用它算相对位移做闭环
#     /vision/box_camera    geometry_msgs/Pose  目标箱子在相机坐标/画面里的实时位置
#     /walle/command        std_msgs/String     命令行下达的指令
#     /gazebo/model_states  (仿真)              抓取成功检测用
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

try:
    from gazebo_msgs.msg import ModelStates
    _HAS_GAZEBO = True
except ImportError:
    _HAS_GAZEBO = False


class SortingFSM:
    def __init__(self):
        rospy.init_node('fsm_node')

        # ---------------- 房间坐标（map 系）----------------
        self.room_b = (rospy.get_param('~room_b_x', -2.5),
                       rospy.get_param('~room_b_y', 2.0))
        self.room_c = (rospy.get_param('~room_c_x', 2.5),
                       rospy.get_param('~room_c_y', -1.9))

        # 桌前搜索位姿（move_base 导航到这里，面向桌子让相机看箱子）
        self.search_x = rospy.get_param('~search_x', 0.6)
        self.search_y = rospy.get_param('~search_y', 1.5)
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
        self.use_grasp_verify = rospy.get_param('~use_grasp_verify', True)
        self.grasp_verify_mode = rospy.get_param('~grasp_verify_mode', 'gazebo')
        self.lift_success_dz = rospy.get_param('~lift_success_dz', 0.05)
        self.detect_timeout = rospy.get_param('~detect_timeout', 20.0)

        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.set_initial_pose_flag = rospy.get_param('~set_initial_pose', True)

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
        rospy.Subscriber('/vision/box_camera', Pose, self.camera_detection_cb)
        rospy.Subscriber('/vision/box_detected', Pose, self.detection_cb)
        rospy.Subscriber('/walle/command', String, self.command_cb)

        # Gazebo 模型状态（仿真抓取成功检测；真机用 joint 模式或关闭）
        self.model_states = None
        self.grasp_verify_srv = None
        if self.use_grasp_verify and self.grasp_verify_mode == 'gazebo' and _HAS_GAZEBO:
            rospy.Subscriber('/gazebo/model_states', ModelStates,
                             self.model_states_cb, queue_size=1)
        elif self.use_grasp_verify and self.grasp_verify_mode == 'joint' and _HAS_TRIGGER:
            try:
                rospy.wait_for_service('/walle/grasp_verify', timeout=10.0)
                self.grasp_verify_srv = rospy.ServiceProxy('/walle/grasp_verify', Trigger)
                rospy.loginfo('已连接真机抓取检测服务 /walle/grasp_verify')
            except rospy.ROSException:
                rospy.logwarn('等待 /walle/grasp_verify 超时，抓取检测将跳过')

        self.busy = False
        self.state = 'idle'

        # ---------------- 导航 ----------------
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('等待 move_base 服务器...')
        self.move_base.wait_for_server()
        rospy.loginfo('已连接 move_base')

        rospy.sleep(1.0)
        if self.set_initial_pose_flag:
            self.publish_initial_pose()

        rospy.loginfo('=' * 50)
        rospy.loginfo('分拣 FSM 就绪，等待指令')
        rospy.loginfo("指令示例: rostopic pub -1 /walle/command std_msgs/String \"red B\"")
        rospy.loginfo('  颜色: red/green/yellow，房间: B 或 C')
        rospy.loginfo('=' * 50)

    # ========================================================
    # 回调
    # ========================================================
    def nav_cmd_cb(self, msg):
        """move_base 的速度只在导航模式下转发到 /cmd_vel；抓取模式下一律忽略。
        这样 /cmd_vel 永远只有 fsm 一个发布者，不会和抓取控制打架。"""
        if self.nav_mode:
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

    def model_states_cb(self, msg):
        self.model_states = msg

    def detection_cb(self, msg):
        color_map = {1.0: 'red', 2.0: 'green', 3.0: 'yellow',
                     4.0: 'blue', 5.0: 'brown', 0.0: 'brown'}
        color = color_map.get(msg.orientation.w, 'brown')
        self.latest_by_color[color] = (msg.position.x, msg.position.y,
                                       msg.position.z, rospy.Time.now())

    def camera_detection_cb(self, msg):
        color_map = {1.0: 'red', 2.0: 'green', 3.0: 'yellow',
                     4.0: 'blue', 5.0: 'brown', 0.0: 'brown'}
        color = color_map.get(msg.orientation.w, 'brown')
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
            self.stop_base()
            self.busy = False
            self.set_state('idle')

    # ========================================================
    # 主流程
    # ========================================================
    def execute_task(self, color, room):
        # 1. 导航到桌前
        self.set_state('goto_table')
        if not self.nav_to(self.search_x, self.search_y, self.search_yaw):
            rospy.logwarn('导航到桌前失败，放弃任务')
            return
        rospy.sleep(1.0)  # 等底盘稳定，pose 不抖

        # 2. 抓取循环（失败重抓）
        grabbed = False
        for attempt in range(1, self.max_grab_attempts + 1):
            rospy.loginfo('======== 第 %d 次抓取 ========', attempt)
            if self.grab_once(color):
                grabbed = True
                break
            rospy.logwarn('第 %d 次抓取失败，回桌前重试', attempt)
            self.nav_to(self.search_x, self.search_y, self.search_yaw)
            rospy.sleep(1.0)

        if not grabbed:
            rospy.logwarn('尝试 %d 次仍未抓到，放弃并退回桌前', self.max_grab_attempts)
            self.reset_arm()
            self.nav_to(self.search_x, self.search_y, self.search_yaw)
            return

        # 3. 送到目标房间
        self.set_state('to_room')
        rx, ry = self.room_b if room == 'B' else self.room_c
        if not self.nav_to(rx, ry, 0.0):
            rospy.logwarn('导航到房间 %s 失败，仍尝试放下', room)

        # 4. 放下
        self.set_state('placing')
        self.place_sequence()

        # 5. 回桌前，等下一条指令
        self.set_state('returning')
        self.nav_to(self.search_x, self.search_y, self.search_yaw)
        rospy.loginfo('任务完成：%s 箱子已送到房间 %s', color or '一个', room)

    def grab_once(self, color):
        """单次抓取：相机闭环对位 -> 闭合 -> 举起 -> 后退。

        抓取控制只依赖 /vision/box_camera 中命令指定颜色的箱子，不使用 map
        坐标。别的颜色/别的箱子再近，也不会影响抓取阈值。
        """
        # 1. 先确认摄像头能看到命令指定颜色的目标箱子
        self.set_state('scanning')
        if self.wait_see_camera_box(color) is None:
            rospy.logwarn('未检测到目标箱子')
            return False

        # 只用于仿真抓取成功检测，不参与对齐/距离阈值控制
        box_map = self.pick_box(color)
        target_model = None
        if box_map is not None:
            target_model = self.nearest_box_model(box_map[0], box_map[1])
        init_z = target_model[3] if target_model else None

        # 2. 抬臂到夹取高度 + 张开夹爪（先备好再对位靠近）
        self.set_state('hand_up')
        rospy.loginfo('抬臂到 %.2fm 并张开夹爪', self.lift_grab_height)
        self.arm_cmd(self.lift_grab_height, self.gripper_open)
        rospy.sleep(2.5)

        # 3. 摄像头闭环：让目标箱子和夹爪中心垂直对齐，并只用该目标深度判断阈值
        self.set_state('align')
        if not self.align_to_camera_box(color):
            rospy.logwarn('闭环对位失败')
            return False

        # 4. 闭合夹爪
        self.set_state('grab')
        rospy.loginfo('闭合夹爪')
        self.arm_cmd(self.lift_grab_height, self.gripper_close)
        rospy.sleep(3.0)

        # 5. 举起箱子（搬运姿态，全程举着）
        rospy.loginfo('举起箱子到搬运高度 %.2fm', self.lift_carry_height)
        self.arm_cmd(self.lift_carry_height, self.gripper_close)
        rospy.sleep(2.5)

        # 6. 后退离开桌子
        self.set_state('backward')
        self.move_rel(-self.back_distance, 0.0)

        # 7. 抓取成功检测
        ok = self.verify_grasp(target_model, init_z)
        if ok:
            rospy.loginfo('抓取成功，举着箱子搬运')
        else:
            rospy.logwarn('抓取检测：手上没有箱子，放下空爪重试')
            self.reset_arm()
        return ok

    # ========================================================
    # 视觉 + 闭环对位
    # ========================================================
    def wait_see_camera_box(self, color):
        """等到摄像头看见目标颜色箱子。"""
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            box = self.camera_box(color)
            if box is not None:
                rospy.loginfo(
                    '看到目标箱子 camera=(x %.3f, depth %.3f, pixel %.3f)',
                    box['cam_x'], box['depth'], box['pixel_x'])
                return box
            if (rospy.Time.now() - start).to_sec() > self.detect_timeout:
                return None
            rate.sleep()
        return None

    def camera_box(self, color):
        """取指定颜色的最新相机检测。color 为空时取任意颜色中面积最大的。"""
        now = rospy.Time.now()
        if color is not None:
            box = self.latest_camera_by_color.get(color)
            if box is None or (now - box['stamp']).to_sec() > 0.8:
                return None
            return box

        candidates = [
            box for box in self.latest_camera_by_color.values()
            if (now - box['stamp']).to_sec() <= 0.8
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda b: b['area'])

    def align_to_camera_box(self, color):
        """相机闭环对位。

        目标：
        - pixel_x 对到 gripper_pixel_x：相当于旋转角度对准，让目标在夹爪中心所在画面列
        - cam_x 对到 gripper_camera_x：横移对齐夹爪中点
        - depth 对到 grab_object_x：只用这个目标箱子的深度作为抓取阈值

        不使用 map 坐标，不看其他箱子的距离。
        """
        target_depth = self.grab_object_x + self.grab_forward_offset
        target_x = self.gripper_camera_x + self.grab_y_offset
        target_pixel_x = self.gripper_pixel_x
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        last_log = rospy.Time.now()

        while not rospy.is_shutdown():
            box = self.camera_box(color)
            if box is None:
                self.stop_base()
                if (rospy.Time.now() - start).to_sec() > self.align_timeout:
                    rospy.logwarn('对位超时：看不到箱子')
                    return False
                rate.sleep()
                continue

            depth = box['depth']
            cam_x = box['cam_x']
            pixel_x = box['pixel_x']
            e_depth = depth - target_depth
            e_lateral = cam_x - target_x
            e_yaw = pixel_x - target_pixel_x

            if (rospy.Time.now() - last_log).to_sec() > 1.0:
                rospy.loginfo(
                    '相机对位中: depth=%.3f e_depth=%.3f cam_x=%.3f e_x=%.3f pixel=%.3f e_yaw=%.3f',
                    depth, e_depth, cam_x, e_lateral, pixel_x, e_yaw)
                last_log = rospy.Time.now()

            if (abs(e_depth) < self.align_tol_depth and
                    abs(e_lateral) < self.align_tol_lateral and
                    abs(e_yaw) < self.align_tol_yaw):
                self.stop_base()
                rospy.loginfo(
                    '相机对位完成: depth=%.3f cam_x=%.3f pixel=%.3f',
                    depth, cam_x, pixel_x)
                return True

            if (rospy.Time.now() - start).to_sec() > self.align_timeout:
                self.stop_base()
                rospy.logwarn(
                    '相机对位超时: depth=%.3f e_depth=%.3f cam_x=%.3f e_x=%.3f pixel=%.3f e_yaw=%.3f',
                    depth, e_depth, cam_x, e_lateral, pixel_x, e_yaw)
                return False

            tw = Twist()
            tw.linear.x = self.clamp(self.align_k_depth * e_depth, self.align_max_v)
            # Camera optical x positive means target appears to the right.
            # Positive base linear.y is left on WPB, so signs are opposite.
            tw.linear.y = self.clamp(-self.align_k_lateral * e_lateral,
                                     self.align_max_v)
            # Rotate so the target image center moves toward the gripper center.
            tw.angular.z = self.clamp(-self.align_k_yaw * e_yaw,
                                      self.align_max_w)
            self.cmd_pub.publish(tw)
            rate.sleep()
        return False

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
    # 抓取成功检测
    # ========================================================
    def nearest_box_model(self, x, y):
        ms = self.model_states
        if ms is None:
            return None
        best, best_d = None, 1e9
        for i, name in enumerate(ms.name):
            if 'box' not in name and 'cube' not in name:
                continue
            p = ms.pose[i].position
            d = (p.x - x) ** 2 + (p.y - y) ** 2
            if d < best_d:
                best_d, best = d, (name, p.x, p.y, p.z)
        return best

    def model_z(self, name):
        ms = self.model_states
        if ms is None:
            return None
        for i, n in enumerate(ms.name):
            if n == name:
                return ms.pose[i].position.z
        return None

    def verify_grasp(self, target_model, init_z):
        if not self.use_grasp_verify or self.grasp_verify_mode == 'none':
            return True
        if self.grasp_verify_mode == 'joint':
            if self.grasp_verify_srv is None:
                rospy.logwarn('未连接真机抓取检测，跳过验证')
                return True
            try:
                resp = self.grasp_verify_srv()
                return resp.success
            except rospy.ServiceException as exc:
                rospy.logwarn('抓取检测服务调用失败: %s', exc)
                return True
        if not _HAS_GAZEBO:
            return True
        if target_model is None or init_z is None or self.model_states is None:
            rospy.logwarn('无法获取箱子模型状态，跳过抓取检测')
            return True
        cur_z = self.model_z(target_model[0])
        if cur_z is None:
            return True
        dz = cur_z - init_z
        rospy.loginfo('抓取检测: %s 高度变化 %.3fm (阈值 %.3f)',
                      target_model[0], dz, self.lift_success_dz)
        return dz > self.lift_success_dz

    # ========================================================
    # 运动：导航(move_base) + /odom 闭环平移
    # ========================================================
    def nav_to(self, x, y, yaw):
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
        self.nav_mode = False         # 先关转发，move_base 之后发的速度全被忽略
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
            self.cmd_pub.publish(tw)
            rate.sleep()
        return False

    def stop_base(self):
        self.cmd_pub.publish(Twist())

    # ========================================================
    # 机械臂
    # ========================================================
    def arm_cmd(self, lift, gripper):
        msg = JointState()
        msg.name = ['lift', 'gripper']
        msg.position = [lift, gripper]
        msg.velocity = [0.5, 5.0]
        for _ in range(10):
            msg.header.stamp = rospy.Time.now()
            self.arm_pub.publish(msg)
            rospy.sleep(0.05)

    def reset_arm(self):
        """机械臂复位：降下 + 张开。"""
        self.arm_cmd(0.0, self.gripper_open)
        rospy.sleep(1.0)

    def place_sequence(self):
        """放置：从搬运高度降到放置高度 -> 张爪释放 -> 后退 -> 复位。"""
        rospy.loginfo('开始放置...')
        # 从搬运高度降到放置高度（保持夹紧）
        self.arm_cmd(self.place_height, self.gripper_close)
        rospy.sleep(2.5)
        # 松爪释放
        self.arm_cmd(self.place_height, self.gripper_open)
        rospy.sleep(1.5)
        # 后退后复位
        self.move_rel(-self.back_distance, 0.0)
        self.reset_arm()
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
        rospy.loginfo('已发布 AMCL 初始位姿 (0,0,0)')

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        SortingFSM().run()
    except rospy.ROSInterruptException:
        rospy.loginfo('FSM 被中断')
