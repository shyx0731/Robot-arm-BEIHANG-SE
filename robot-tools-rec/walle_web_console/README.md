# walle_web_console

浏览器版 Walle 分拣控制台，对接 **walle_core** 后端（功能对齐 `rqt_task_dashboard`）。

## 依赖

```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-rosapi \
  ros-noetic-image-transport ros-noetic-image-transport-plugins
```

## 启动

```bash
# 终端 1：分拣系统（仿真示例）
roslaunch walle_gazebo_assets walle_circumstances_with_boxes.launch

# 终端 2：Web 控制台
roslaunch walle_web_console web_ui.launch
```

浏览器打开：**http://localhost:8080**，点击「连接 ROS」（默认 `ws://localhost:9090`）。

## ROS 接口

| 功能 | 话题 |
|------|------|
| 下发任务 | 发布 `/walle/command`，如 `red B` |
| 状态 | 订阅 `/task/status` |
| 检测 | 订阅 `/vision/box_camera` |
| 画面 | 订阅 `/vision/debug_image/compressed` |
| 急停 | 发布 `/cmd_vel` 零速度 |

## 文件

| 路径 | 说明 |
|------|------|
| `web/` | 前端静态资源（HTML / CSS / JS） |
| `scripts/web_server.py` | HTTP 静态文件服务 |
| `launch/web_ui.launch` | rosbridge + 页面 + 视频压缩转码 |
