# walle_gazebo_assets

仿真分拣场景资源：`walle_core` 的 launch 引用 `wpr_simulation/models/cubes/`，但官方包没有这些 SDF，导致 Gazebo 桌上没有彩色箱子。

本包提供红/绿/黄立方体模型，并在 Gazebo 启动后补生成。
这个入口只负责场景，不启动 `walle_core` 的控制器/业务节点。

## 启动

```bash
# 推荐：仿真场景 + 箱子一次启动
roslaunch walle_gazebo_assets walle_circumstances_with_boxes.launch

# 或：已有 walle_circumstances 在跑，只补箱子
roslaunch walle_gazebo_assets spawn_sorting_cubes.launch
```

## 模型

| 文件 | 说明 |
|------|------|
| `models/cubes/red_cube.sdf` | 红色 10cm 立方体 |
| `models/cubes/green_cube.sdf` | 绿色 |
| `models/cubes/yellow_cube.sdf` | 黄色 |

桌子位置 `(1.5, 1.5)`，9 个箱子分布在 x=1.4~1.6, y=1.1~1.9, z=0.79。
