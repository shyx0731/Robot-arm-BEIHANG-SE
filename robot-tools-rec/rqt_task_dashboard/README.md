# rqt_task_dashboard — Walle 分拣控制台

对接 **walle_core** 后端的人机交互面板（基于原 robot-tools-rec 前端改造）。

## 功能

| 模块 | ROS 接口 | 说明 |
|------|----------|------|
| 分拣控制 | 发布 `/walle/command` | 如 `red B`、`green C` |
| 运行状态 | 订阅 `/task/status` | FSM 状态实时显示（中文） |
| 里程计 | 订阅 `/odom` | 位姿 x/y/yaw |
| 视觉检测 | 订阅 `/vision/box_camera` | 红/绿/黄箱子深度与面积 |
| 视频回显 | 订阅 `/vision/debug_image` | 实时调试画面 |
| 急停 | 发布 `/cmd_vel` 零速度 | 安全急停 |
| 任务记录 | 本地表格 | 指令历史 + 成功率统计 |

## 依赖

```bash
sudo apt install ros-noetic-rqt-gui ros-noetic-rqt-gui-py \
  python3-opencv python3-pyqtgraph
```

## 编译

将 `robot-tools-rec/rqt_task_dashboard` 与 `walle_core` 一并放入 catkin_ws：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 终端 1：仿真 + 桌上彩色箱子
roslaunch rqt_task_dashboard walle_circumstances_with_boxes.launch

# 或队友原版（需 wpr_simulation/models/cubes/ 存在，见下文）
# roslaunch walle_core walle_circumstances.launch
# roslaunch rqt_task_dashboard spawn_sorting_cubes.launch   # 另开终端补箱子

# 终端 2：RQT 桌面控制台
roslaunch rqt_task_dashboard walle_dashboard.launch

# 终端 3：Web 浏览器控制台
roslaunch rqt_task_dashboard web_ui.launch
# → http://localhost:8080 → 点击「连接 ROS」
```

## 彩色箱子说明

队友在 `walle_core/launch/includes/sim_gazebo.launch` 里用 `gazebo_ros spawn_model` 部署箱子，
路径为 `wpr_simulation/models/cubes/*.sdf`（需在本机 wpr_simulation 包里手动创建，官方 GitHub 无此目录）。

本包提供：
- `models/cubes/` — 红/绿/黄 SDF 模型
- `launch/spawn_sorting_cubes.launch` — Gazebo 启动后补生成 9 个箱子
- `launch/walle_circumstances_with_boxes.launch` — 仿真 + 补箱子一键启动

## 其他 launch

| 命令 | 说明 |
|------|------|
| `roslaunch rqt_task_dashboard spawn_sorting_cubes.launch` | 仅补 spawn 箱子 |
| `roslaunch rqt_task_dashboard web_ui.launch` | rosbridge + Web 页面 + 视频压缩 |
| `roslaunch rqt_task_dashboard walle_dashboard.launch` | RQT 桌面控制台 |

## 可配置参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `command_topic` | `/walle/command` | 分拣指令 |
| `task_status_topic` | `/task/status` | FSM 状态 |
| `robot_pose_topic` | `/odom` | 里程计 |
| `video_topic` | `/vision/debug_image` | 视觉画面 |
| `box_camera_topic` | `/vision/box_camera` | 箱子检测 |
| `cmd_vel_topic` | `/cmd_vel` | 急停 |
| `video_max_fps` | `15.0` | 视频限帧 |

## 与旧版差异

旧版使用 `/task_manager/append_task` 服务和 `/task_status` 话题；  
新版直接对接 walle_core 的 `/walle/command` + `/task/status`，无需额外 task_manager 节点。
