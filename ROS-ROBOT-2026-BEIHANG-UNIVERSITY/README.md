# ROS-ROBOT-2026-BEIHANG-UNIVERSITY

WPB Home 仓储分拣机器人：仿真 + 真机（Sim2Real）统一工程。

基于 ROS Noetic，实现「导航 → 相机识别 → 闭环对位 → 抓取 → 按颜色分拣到房间 B/C」全流程，支持命令行下发任务。

---

## 功能概览

- 彩色箱子检测（红 / 绿 / 黄），Kinect 彩色 + 深度
- **相机闭环抓取**：不用地图坐标对位，只跟踪命令指定颜色的箱子
- `move_base` 导航 + AMCL 定位
- 仿真与真机业务代码分离（`stimulation/` vs `real/`），参数文件可互通
- 命令行控制示例：

```bash
rostopic pub -1 /walle/command std_msgs/String "red B"
```

---

## 环境依赖

| 组件 | 说明 |
|------|------|
| Ubuntu 20.04 | |
| ROS Noetic | |
| Gazebo | 仿真 |
| catkin 依赖包 | `wpb_home_bringup`、`wpb_home_tutorials`、`wpr_simulation`、`rplidar_ros`、`iai_kinect2` |

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 仓库结构

```
src/
├── walle_core/              # 主功能包
│   ├── src/stimulation/     # 仿真专用节点
│   ├── src/real/            # 真机专用节点
│   ├── config/              # 仿真 / 真机参数 yaml
│   ├── launch/              # 各场景 launch
│   └── maps/                # my_map（真机）/ wpb_gmapping_map（仿真）
└── walle_msgs/              # 自定义消息
```

---

## 启动方式

### 1. 纯仿真（Gazebo + 仿真地图 + 桌子 + 箱子）

```bash
roslaunch walle_core walle_circumstances.launch
```

- 地图：`maps/wpb_gmapping_map.yaml`
- 代码：`src/stimulation/`
- 自动发布初始位姿，无需手动画 2D Pose Estimate

### 2. 仿真里调试真机逻辑（空 Gazebo + my_map + real 代码）

```bash
roslaunch walle_core walle_real_in_sim.launch
```

- Gazebo：**空地面 + 机器人**（不生成仿真桌子、仿真彩色箱子）
- 地图：`maps/my_map.yaml`
- 代码：`src/real/`
- 参数：`config/my_map_sorting.yaml`

### 3. 真机

```bash
roslaunch walle_core walle_real.launch
```

**每次真机启动建议流程：**

1. 确认 `/dev/ftdi`（底盘）、`/dev/rplidar`（激光）可用  
2. 启动 launch，等 FSM 打印「真机分拣 FSM 就绪」  
3. RViz → **2D Pose Estimate** → 点机器人真实位置（**每次启动做一次**）  
4. 发分拣指令：

```bash
rostopic pub -1 /walle/command std_msgs/String "red B"
```

> 启动后、发指令前，机器人默认**不会自己乱走**；没做 2D Pose Estimate 就发指令，会按**错误位姿**导航。

---

## 导航点标定（真机，一般做一次）

编辑 `src/walle_core/config/my_map_sorting.yaml`：

| 参数 | 含义 |
|------|------|
| `search_x` / `search_y` / `search_yaw` | 桌前搜索位（先导航到这里，再抓取） |
| `room_b_x` / `room_b_y` | 房间 B 放置点 |
| `room_c_x` / `room_c_y` | 房间 C 放置点 |

**标定方法：**

1. 启动 `walle_real.launch`，完成 2D Pose Estimate  
2. RViz 工具栏 **Publish Point**，在地图上点击目标位置  
3. 终端会打印 map 坐标，例如：`Point clicked at (1.23, 4.56, 0.0) in frame map`  
4. 把 x、y 写入 yaml，保存后**重新 roslaunch**

| 操作 | 频率 |
|------|------|
| Publish Point 标定导航点 → 写入 yaml | **一次**（环境变了再标） |
| RViz 2D Pose Estimate 定位 | **真机每次启动** |

---

## 参数文件说明

| 文件 | 用途 |
|------|------|
| `config/sim_sorting.yaml` | 仿真（wpb_gmapping_map） |
| `config/my_map_sorting.yaml` | 真机 + sim2real 调试（my_map） |
| `config/my_map_sim_hw_overlay.yaml` | 仿真调试时覆盖 Kinect 深度话题 |

抓取微调（仿真 / 真机通用，写在 `my_map_sorting.yaml`）：

- `grab_object_x`：目标箱子深度阈值（主距离旋钮）
- `lift_grab_height` / `lift_carry_height`：夹取 / 搬运高度
- `gripper_camera_x` / `gripper_pixel_x`：夹爪与画面对齐微调

---

## 急停与安全

真机异常（摔倒、定位飘了、行为不对）时立即：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{}"
```

当前代码**没有摔倒检测、没有自动急停**。摔倒后需：

1. 人工急停 / 扶起机器人  
2. RViz 重新 **2D Pose Estimate**  
3. 确认机械臂无异常后再继续  

---

## 分支说明

| 分支 | 说明 |
|------|------|
| `main` | 主分支 |
| `dev` | 开发分支 |
| `feature/*` | 功能分支 |

---

## 提交规范（Commit Message）

| type | 说明 |
|------|------|
| feat | 新功能 |
| fix | Bug 修复 |
| docs | 文档更新 |
| refactor | 代码重构 |
| test | 测试相关 |
| chore | 构建 / 工具相关 |

示例：

```
feat(walle): 添加真机分拣 launch
fix(vision): 修复深度图话题配置
docs(readme): 更新启动说明
```

---

## 学校 / 课程

北京航空航天大学 · 嵌入式软件 2026

---

## 仓库地址

- GitHub：`https://github.com/hz2839788401-sketch/ROS-ROBOT-2026-BEIHANG-UNIVERSITY`

---

## License

课程项目，仅供学习交流使用。
