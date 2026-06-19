#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Walle 分拣控制台 — 对接 walle_core 后端（/walle/command、/task/status 等）。"""

import csv
import math
import time
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from python_qt_binding import QtCore, QtGui, QtWidgets
from qt_gui.plugin import Plugin
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

try:
    import pyqtgraph as pg
except ImportError:  # pragma: no cover
    pg = None


STATE_LABELS = {
    'idle': '空闲',
    'goto_table': '前往桌前',
    'scanning': '扫描箱子',
    'hand_up': '抬臂准备',
    'align': '相机对位',
    'grab': '抓取中',
    'backward': '后退',
    'to_room': '前往房间',
    'placing': '放置中',
    'returning': '返回桌前',
}

COLOR_CODE = {1.0: 'red', 2.0: 'green', 3.0: 'yellow', 4.0: 'blue', 5.0: 'brown'}
COLOR_CN = {'red': '红色', 'green': '绿色', 'yellow': '黄色', 'blue': '蓝色', 'brown': '棕色'}


@dataclass
class TaskItem:
    task_id: str
    color: str
    room: str
    status: str


@dataclass
class DetectionInfo:
    depth: float = 0.0
    area: float = 0.0
    stamp: float = 0.0


class TaskDashboard(Plugin):
    def __init__(self, context):
        super(TaskDashboard, self).__init__(context)
        self.setObjectName('TaskDashboard')

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_status = 'idle'
        self._latest_pose = 'x=0.00, y=0.00, yaw=0.00'
        self._latest_battery = 'N/A'
        self._latest_frame = None
        self._last_frame_ts = 0.0
        self._fps = 0.0
        self._tasks: List[TaskItem] = []
        self._task_counter = 1
        self._active_task_id: Optional[str] = None
        self._was_busy = False
        self._detections: Dict[str, DetectionInfo] = {
            'red': DetectionInfo(),
            'green': DetectionInfo(),
            'yellow': DetectionInfo(),
        }

        self._command_topic = rospy.get_param('~command_topic', '/walle/command')
        self._task_status_topic = rospy.get_param('~task_status_topic', '/task/status')
        self._robot_pose_topic = rospy.get_param('~robot_pose_topic', '/odom')
        self._battery_topic = rospy.get_param('~battery_state_topic', '/battery_state')
        self._video_topic = rospy.get_param('~video_topic', '/vision/debug_image')
        self._box_camera_topic = rospy.get_param('~box_camera_topic', '/vision/box_camera')
        self._cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self._stats_file = rospy.get_param('~stats_file', '')
        self._video_max_fps = float(rospy.get_param('~video_max_fps', 15.0))

        self._widget = QtWidgets.QWidget()
        self._widget.setWindowTitle('Walle 分拣控制台')
        self._build_ui(self._widget)
        self._apply_theme()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                '{} ({})'.format(self._widget.windowTitle(), context.serial_number()))
        context.add_widget(self._widget)

        self._cmd_pub = rospy.Publisher(self._command_topic, String, queue_size=5)
        self._cmd_vel_pub = rospy.Publisher(self._cmd_vel_topic, Twist, queue_size=5)

        self._status_sub = rospy.Subscriber(
            self._task_status_topic, String, self._on_status, queue_size=10)
        self._pose_sub = rospy.Subscriber(
            self._robot_pose_topic, Odometry, self._on_odom, queue_size=10)
        self._battery_sub = rospy.Subscriber(
            self._battery_topic, BatteryState, self._on_battery, queue_size=10)
        self._video_sub = rospy.Subscriber(
            self._video_topic, Image, self._on_video, queue_size=1)
        self._box_sub = rospy.Subscriber(
            self._box_camera_topic, Pose, self._on_box_camera, queue_size=10)

        self._ui_timer = QtCore.QTimer(self._widget)
        self._ui_timer.timeout.connect(self._refresh_ui)
        self._ui_timer.start(100)

        self._stats_timer = QtCore.QTimer(self._widget)
        self._stats_timer.timeout.connect(self._refresh_stats)
        self._stats_timer.start(2000)

        self._stop_timer = QtCore.QTimer(self._widget)
        self._stop_timer.timeout.connect(self._publish_stop_tick)
        self._stop_count = 0

        rospy.loginfo('Walle 控制台已启动')
        rospy.loginfo('  指令: %s', self._command_topic)
        rospy.loginfo('  状态: %s', self._task_status_topic)
        rospy.loginfo('  视频: %s', self._video_topic)

    def _build_ui(self, parent):
        root = QtWidgets.QVBoxLayout(parent)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        header = QtWidgets.QWidget()
        header_l = QtWidgets.QHBoxLayout(header)
        header_l.setContentsMargins(8, 8, 8, 8)
        title = QtWidgets.QLabel('Walle 仓储分拣控制台')
        title.setObjectName('HeaderTitle')
        subtitle = QtWidgets.QLabel('北航 2026 · 实时状态 · 抓取控制 · 视觉回显')
        subtitle.setObjectName('HeaderSubtitle')
        title_box = QtWidgets.QVBoxLayout()
        title_box.setSpacing(0)
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header_l.addLayout(title_box)
        header_l.addStretch(1)

        self.badge_ros = QtWidgets.QLabel('ROS Online')
        self.badge_ros.setObjectName('BadgeOk')
        header_l.addWidget(self.badge_ros)
        root.addWidget(header)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        splitter.addWidget(left)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)
        splitter.addWidget(right)
        splitter.setSizes([580, 540])

        # ---- 状态面板 ----
        status_group = QtWidgets.QGroupBox('运行状态')
        status_group.setObjectName('Card')
        sg = QtWidgets.QGridLayout(status_group)
        sg.setContentsMargins(12, 12, 12, 12)
        sg.setHorizontalSpacing(12)
        sg.setVerticalSpacing(10)

        self.lbl_state = QtWidgets.QLabel('空闲')
        self.lbl_state_lamp = QtWidgets.QLabel()
        self.lbl_state_lamp.setFixedSize(16, 16)
        self.lbl_pose = QtWidgets.QLabel('x=0.00, y=0.00, yaw=0.00')
        self.lbl_battery = QtWidgets.QLabel('N/A')
        self.lbl_state.setObjectName('StateText')

        sg.addWidget(QtWidgets.QLabel('FSM 状态'), 0, 0)
        state_box = QtWidgets.QHBoxLayout()
        state_box.addWidget(self.lbl_state_lamp)
        state_box.addWidget(self.lbl_state)
        state_box.addStretch(1)
        sg.addLayout(state_box, 0, 1)
        sg.addWidget(QtWidgets.QLabel('里程计位姿'), 1, 0)
        sg.addWidget(self.lbl_pose, 1, 1)
        sg.addWidget(QtWidgets.QLabel('电量'), 2, 0)
        sg.addWidget(self.lbl_battery, 2, 1)
        sg.setColumnStretch(1, 1)
        left_layout.addWidget(status_group)

        # ---- 视觉检测 ----
        det_group = QtWidgets.QGroupBox('箱子检测（实时）')
        det_group.setObjectName('Card')
        dg = QtWidgets.QHBoxLayout(det_group)
        dg.setContentsMargins(12, 12, 12, 12)
        dg.setSpacing(10)
        self.lbl_det_red = QtWidgets.QLabel('红色: 未检测')
        self.lbl_det_green = QtWidgets.QLabel('绿色: 未检测')
        self.lbl_det_yellow = QtWidgets.QLabel('黄色: 未检测')
        for lbl in (self.lbl_det_red, self.lbl_det_green, self.lbl_det_yellow):
            lbl.setObjectName('DetLabel')
            dg.addWidget(lbl)
        left_layout.addWidget(det_group)

        # ---- 分拣控制 ----
        cmd_group = QtWidgets.QGroupBox('抓取 / 分拣控制')
        cmd_group.setObjectName('Card')
        cg = QtWidgets.QVBoxLayout(cmd_group)
        cg.setContentsMargins(12, 12, 12, 12)
        cg.setSpacing(10)

        controls = QtWidgets.QHBoxLayout()
        controls.setSpacing(8)
        self.cmb_color = QtWidgets.QComboBox()
        self.cmb_color.addItems(['red', 'green', 'yellow'])
        self.cmb_room = QtWidgets.QComboBox()
        self.cmb_room.addItems(['B', 'C'])
        self.cmb_filter = QtWidgets.QComboBox()
        self.cmb_filter.addItems(['全部', 'Queued', 'Running', 'Success', 'Failed'])
        self.cmb_filter.currentTextChanged.connect(self._apply_task_filter)

        self.btn_send = QtWidgets.QPushButton('发送分拣指令')
        self.btn_send.setObjectName('BtnPrimary')
        self.btn_send.clicked.connect(self._on_send_command)

        self.btn_stop = QtWidgets.QPushButton('急停')
        self.btn_stop.setObjectName('BtnDanger')
        self.btn_stop.clicked.connect(self._on_emergency_stop)

        self.btn_clear = QtWidgets.QPushButton('清空记录')
        self.btn_clear.setObjectName('BtnSecondary')
        self.btn_clear.clicked.connect(self._on_clear_tasks)

        controls.addWidget(QtWidgets.QLabel('颜色'))
        controls.addWidget(self.cmb_color)
        controls.addWidget(QtWidgets.QLabel('房间'))
        controls.addWidget(self.cmb_room)
        controls.addWidget(QtWidgets.QLabel('筛选'))
        controls.addWidget(self.cmb_filter)
        controls.addStretch(1)
        controls.addWidget(self.btn_clear)
        controls.addWidget(self.btn_stop)
        controls.addWidget(self.btn_send)
        cg.addLayout(controls)

        quick = QtWidgets.QHBoxLayout()
        quick.setSpacing(6)
        for label, color, room in (
            ('红→B', 'red', 'B'), ('红→C', 'red', 'C'),
            ('绿→B', 'green', 'B'), ('绿→C', 'green', 'C'),
            ('黄→B', 'yellow', 'B'), ('黄→C', 'yellow', 'C'),
        ):
            btn = QtWidgets.QPushButton(label)
            btn.setObjectName('BtnQuick')
            btn.clicked.connect(lambda _=False, c=color, r=room: self._send_command(c, r))
            quick.addWidget(btn)
        cg.addLayout(quick)

        self.task_table = QtWidgets.QTableWidget(0, 4)
        self.task_table.setObjectName('TaskTable')
        self.task_table.setHorizontalHeaderLabels(['ID', '颜色', '房间', '状态'])
        self.task_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.task_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.task_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.task_table.setAlternatingRowColors(True)
        self.task_table.verticalHeader().setVisible(False)
        cg.addWidget(self.task_table)
        left_layout.addWidget(cmd_group)

        # ---- 统计 ----
        stats_group = QtWidgets.QGroupBox('任务统计')
        stats_group.setObjectName('Card')
        stg = QtWidgets.QVBoxLayout(stats_group)
        stg.setContentsMargins(12, 12, 12, 12)
        summary = QtWidgets.QHBoxLayout()
        self.lbl_total = QtWidgets.QLabel('总任务数: 0')
        self.lbl_success = QtWidgets.QLabel('成功率: 0.00%')
        self.lbl_total.setObjectName('Kpi')
        self.lbl_success.setObjectName('Kpi')
        summary.addWidget(self.lbl_total)
        summary.addWidget(self.lbl_success)
        summary.addStretch(1)
        self.btn_refresh_stats = QtWidgets.QPushButton('刷新统计')
        self.btn_refresh_stats.setObjectName('BtnSecondary')
        self.btn_refresh_stats.clicked.connect(self._refresh_stats)
        summary.addWidget(self.btn_refresh_stats)
        stg.addLayout(summary)

        if pg:
            self.chart = pg.PlotWidget()
            self.chart.setBackground(None)
            self.chart.setLabel('left', '任务数量')
            self.chart.setLabel('bottom', '类别')
            self.chart.showGrid(x=True, y=True, alpha=0.2)
            stg.addWidget(self.chart)
        else:
            self.chart = None
            tip = QtWidgets.QLabel('未安装 pyqtgraph，图表已禁用')
            tip.setObjectName('Muted')
            stg.addWidget(tip)
        right_layout.addWidget(stats_group, stretch=1)

        # ---- 视频 ----
        video_group = QtWidgets.QGroupBox('视觉调试画面（实时）')
        video_group.setObjectName('Card')
        vg = QtWidgets.QVBoxLayout(video_group)
        vg.setContentsMargins(12, 12, 12, 12)
        self.video_label = QtWidgets.QLabel('等待 /vision/debug_image …')
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumSize(480, 360)
        self.video_label.setObjectName('VideoLabel')
        vg.addWidget(self.video_label)
        foot = QtWidgets.QHBoxLayout()
        self.lbl_video_topic = QtWidgets.QLabel(self._video_topic)
        self.lbl_video_topic.setObjectName('Muted')
        self.lbl_fps = QtWidgets.QLabel('FPS: 0.0')
        self.lbl_fps.setObjectName('Muted')
        foot.addWidget(self.lbl_video_topic)
        foot.addStretch(1)
        foot.addWidget(self.lbl_fps)
        vg.addLayout(foot)
        right_layout.addWidget(video_group, stretch=2)

        left_layout.addStretch(1)

    def _apply_theme(self):
        qss = """
        QWidget {
          font-family: "Noto Sans CJK SC","Microsoft YaHei","Ubuntu";
          font-size: 11pt;
          color: #e6e6e6;
          background: #0f172a;
        }
        QGroupBox#Card {
          background: rgba(255,255,255,0.06);
          border: 1px solid rgba(255,255,255,0.10);
          border-radius: 12px;
          margin-top: 12px;
        }
        QGroupBox#Card::title {
          subcontrol-origin: margin;
          left: 12px;
          padding: 0 8px;
          color: #cbd5e1;
          font-weight: 600;
        }
        QLabel#HeaderTitle { font-size: 18pt; font-weight: 800; color: #f8fafc; }
        QLabel#HeaderSubtitle { color: #94a3b8; }
        QLabel#Muted { color: #94a3b8; }
        QLabel#Kpi { font-size: 12pt; font-weight: 700; color: #f8fafc; }
        QLabel#StateText { font-size: 18pt; font-weight: 800; }
        QLabel#DetLabel {
          padding: 8px 12px;
          border-radius: 8px;
          background: rgba(255,255,255,0.05);
          border: 1px solid rgba(255,255,255,0.08);
        }
        QLabel#BadgeOk {
          padding: 4px 10px; border-radius: 999px;
          background: rgba(34,197,94,0.18);
          border: 1px solid rgba(34,197,94,0.35);
          color: #86efac; font-weight: 700;
        }
        QLabel#BadgeBad {
          padding: 4px 10px; border-radius: 999px;
          background: rgba(239,68,68,0.16);
          border: 1px solid rgba(239,68,68,0.35);
          color: #fecaca; font-weight: 700;
        }
        QComboBox {
          background: rgba(255,255,255,0.07);
          border: 1px solid rgba(255,255,255,0.10);
          border-radius: 8px; padding: 6px 10px;
        }
        QPushButton {
          border: 1px solid rgba(255,255,255,0.12);
          border-radius: 10px; padding: 8px 14px;
          background: rgba(255,255,255,0.06);
        }
        QPushButton:hover { background: rgba(255,255,255,0.10); }
        QPushButton#BtnPrimary {
          background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #22c55e, stop:1 #16a34a);
          border: 1px solid rgba(34,197,94,0.55);
          color: #06230f; font-weight: 800;
        }
        QPushButton#BtnSecondary {
          background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #60a5fa, stop:1 #3b82f6);
          border: 1px solid rgba(59,130,246,0.55);
          color: #061426; font-weight: 800;
        }
        QPushButton#BtnDanger {
          background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #fb7185, stop:1 #ef4444);
          border: 1px solid rgba(239,68,68,0.55);
          color: #2b050a; font-weight: 800;
        }
        QPushButton#BtnQuick {
          font-size: 9pt; padding: 6px 8px;
        }
        QTableWidget#TaskTable {
          background: rgba(255,255,255,0.04);
          border: 1px solid rgba(255,255,255,0.10);
          border-radius: 10px;
          gridline-color: rgba(255,255,255,0.10);
        }
        QHeaderView::section {
          background: rgba(148,163,184,0.14);
          color: #e2e8f0; padding: 8px; border: none; font-weight: 800;
        }
        QTableWidget::item:selected { background: rgba(96,165,250,0.28); }
        QLabel#VideoLabel {
          background: #0b1020;
          border: 1px solid rgba(255,255,255,0.10);
          border-radius: 10px; color: #94a3b8;
        }
        """
        self._widget.setStyleSheet(qss)

    def _on_status(self, msg):
        raw = msg.data.strip()
        with self._lock:
            self._latest_status = raw
        self._sync_task_from_fsm(raw)

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with self._lock:
            self._latest_pose = 'x={:.2f}, y={:.2f}, yaw={:.2f}'.format(x, y, yaw)

    def _on_battery(self, msg):
        percent = msg.percentage * 100.0 if msg.percentage >= 0.0 else 0.0
        with self._lock:
            self._latest_battery = '{:.2f} V ({:.1f}%)'.format(msg.voltage, percent)

    def _on_box_camera(self, msg):
        color = COLOR_CODE.get(msg.orientation.w)
        if color not in self._detections:
            return
        with self._lock:
            self._detections[color] = DetectionInfo(
                depth=msg.position.z,
                area=msg.orientation.z,
                stamp=time.time(),
            )

    def _on_video(self, msg):
        try:
            now = time.time()
            if self._video_max_fps > 0:
                min_dt = 1.0 / self._video_max_fps
                if self._last_frame_ts > 0 and (now - self._last_frame_ts) < min_dt:
                    return
            if self._last_frame_ts > 0.0:
                dt = now - self._last_frame_ts
                if dt > 0:
                    measured = 1.0 / dt
                    self._fps = 0.8 * self._fps + 0.2 * measured if self._fps > 0 else measured
            self._last_frame_ts = now
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._latest_frame = frame
        except CvBridgeError as err:
            rospy.logwarn_throttle(2.0, 'Video conversion failed: %s', err)

    def _sync_task_from_fsm(self, raw_status):
        is_idle = raw_status == 'idle'
        if not is_idle:
            self._was_busy = True
            if self._active_task_id:
                for item in self._tasks:
                    if item.task_id == self._active_task_id and item.status == 'Queued':
                        item.status = 'Running'
                        self._refresh_task_table()
            return
        if self._was_busy and self._active_task_id:
            for item in self._tasks:
                if item.task_id == self._active_task_id:
                    item.status = 'Success'
                    break
            self._active_task_id = None
            self._was_busy = False
            self._refresh_task_table()
            self._refresh_stats()

    def _refresh_ui(self):
        with self._lock:
            status = self._latest_status
            pose = self._latest_pose
            battery = self._latest_battery
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            dets = {k: DetectionInfo(v.depth, v.area, v.stamp)
                    for k, v in self._detections.items()}

        label = STATE_LABELS.get(status, status)
        self.lbl_state.setText(label)
        self.lbl_pose.setText(pose)
        self.lbl_battery.setText(battery)
        self.lbl_fps.setText('FPS: {:.1f}'.format(self._fps))

        color = '#22c55e'
        if status == 'idle':
            color = '#22c55e'
        elif status in ('align', 'grab', 'scanning'):
            color = '#f59e0b'
        else:
            color = '#3b82f6'
        self.lbl_state.setStyleSheet('color: {};'.format(color))
        self.lbl_state_lamp.setStyleSheet(
            'background-color: {}; border-radius: 8px; border: 1px solid #303030;'.format(color))

        now = time.time()
        for color_key, lbl in (
            ('red', self.lbl_det_red),
            ('green', self.lbl_det_green),
            ('yellow', self.lbl_det_yellow),
        ):
            d = dets[color_key]
            cn = COLOR_CN.get(color_key, color_key)
            if d.stamp > 0 and (now - d.stamp) < 1.5:
                lbl.setText('{}: {:.2f}m · 面积{:.0f}'.format(cn, d.depth, d.area))
                lbl.setStyleSheet('color: #86efac;')
            else:
                lbl.setText('{}: 未检测'.format(cn))
                lbl.setStyleSheet('color: #94a3b8;')

        try:
            ok = rospy.get_time() > 0.0 and rospy.core.is_initialized()
        except Exception:
            ok = False
        self.badge_ros.setText('ROS Online' if ok else 'ROS Offline')
        self.badge_ros.setObjectName('BadgeOk' if ok else 'BadgeBad')

        if frame is not None:
            self._render_frame(frame)

    def _render_frame(self, frame):
        h, w, _ = frame.shape
        max_w = max(self.video_label.width(), 1)
        max_h = max(self.video_label.height(), 1)
        scale = min(float(max_w) / float(w), float(max_h) / float(h), 1.0)
        out = cv2.resize(frame, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)
        qimg = QtGui.QImage(
            rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888)
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(qimg))

    def _on_send_command(self):
        color = self.cmb_color.currentText()
        room = self.cmb_room.currentText()
        self._send_command(color, room)

    def _send_command(self, color, room):
        payload = '{} {}'.format(color, room)
        self._cmd_pub.publish(String(data=payload))

        task_id = 'T{:04d}'.format(self._task_counter)
        self._task_counter += 1
        self._active_task_id = task_id
        self._was_busy = False
        self._tasks.append(TaskItem(task_id=task_id, color=color, room=room, status='Queued'))
        self._refresh_task_table()
        self._refresh_stats()
        rospy.loginfo('已发送分拣指令: %s (任务 %s)', payload, task_id)

    def _on_emergency_stop(self):
        self._stop_count = 0
        if not self._stop_timer.isActive():
            self._stop_timer.start(50)
        rospy.logwarn('急停中（/cmd_vel 零速度）')

    def _publish_stop_tick(self):
        self._cmd_vel_pub.publish(Twist())
        self._stop_count += 1
        if self._stop_count >= 20:
            self._stop_timer.stop()

    def _refresh_stats(self):
        total, success = self._load_stats()
        ratio = (float(success) / float(total) * 100.0) if total > 0 else 0.0
        self.lbl_total.setText('总任务数: {}'.format(total))
        self.lbl_success.setText('成功率: {:.2f}%'.format(ratio))
        if self.chart:
            self.chart.clear()
            fail = max(total - success, 0)
            bars = pg.BarGraphItem(
                x=[1, 2], height=[success, fail], width=0.6,
                brushes=['#2e7d32', '#c62828'])
            self.chart.addItem(bars)
            self.chart.getAxis('bottom').setTicks([[(1, '成功'), (2, '失败')]])

    def _load_stats(self):
        if not self._stats_file:
            total = len(self._tasks)
            success = sum(1 for t in self._tasks if t.status == 'Success')
            return total, success
        try:
            total = success = 0
            with open(self._stats_file, 'r', encoding='utf-8') as fp:
                for row in csv.DictReader(fp):
                    total += 1
                    if row.get('status', '').lower() == 'success':
                        success += 1
            return total, success
        except Exception as err:
            rospy.logwarn_throttle(5.0, 'Load stats failed: %s', err)
            return len(self._tasks), 0

    def _append_task_row(self, item):
        row = self.task_table.rowCount()
        self.task_table.insertRow(row)
        self.task_table.setItem(row, 0, QtWidgets.QTableWidgetItem(item.task_id))
        self.task_table.setItem(
            row, 1, QtWidgets.QTableWidgetItem(COLOR_CN.get(item.color, item.color)))
        self.task_table.setItem(row, 2, QtWidgets.QTableWidgetItem('房间 ' + item.room))
        self.task_table.setItem(row, 3, QtWidgets.QTableWidgetItem(item.status))

    def _refresh_task_table(self):
        self.task_table.setRowCount(0)
        filt = self.cmb_filter.currentText()
        for item in self._tasks:
            if filt != '全部' and item.status != filt:
                continue
            self._append_task_row(item)

    def _apply_task_filter(self, *_args):
        self._refresh_task_table()

    def _on_clear_tasks(self):
        self._tasks = []
        self._active_task_id = None
        self._was_busy = False
        self.task_table.setRowCount(0)
        self._refresh_stats()

    def shutdown_plugin(self):
        for sub in (self._status_sub, self._pose_sub, self._battery_sub,
                    self._video_sub, self._box_sub):
            try:
                sub.unregister()
            except Exception:
                pass
        if self._ui_timer.isActive():
            self._ui_timer.stop()
        if self._stats_timer.isActive():
            self._stats_timer.stop()
        if self._stop_timer.isActive():
            self._stop_timer.stop()
