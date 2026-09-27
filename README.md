# zhihuishequ-
智慧社区ROS1参考代码

## 红绿灯识别（2026-09-26）

- [ROS Noetic 节点与调用说明](traffic_light_detector/README.md)：相机图像检测、绿灯连续确认、check/reset 服务。
- [本地图片测试](tools/traffic_light_local/使用说明.md)：直接读取图片验证传统 CV 检测。
- [复赛完赛规划与阶段进度](docs/智慧社区复赛完赛规划.md)。

识别节点已在用户仿真环境中输出红灯不可通行、绿灯待确认及绿灯可通行。导航停车、运动互锁以及黄灯/断流/暂停等场景仍需联调验证。

## 第一处路口联调（2026-09-27）

`robot_navigation/scripts/single_intersection_mission.py` 已把第一处路口串成完整状态流程：导航到停止线前观察点、停车、重置红绿灯确认、等待稳定绿灯，再导航到路口后目标点。任一导航失败、服务不可用或等待超时都会取消导航并持续发送零速度。

实测任务点已写入 `robot_navigation/launch/single_intersection_mission.launch`：

| 点位 | x | y | qz | qw |
|---|---:|---:|---:|---:|
| 停止线前 | 1.585639 | 0.501283 | -0.017359 | 0.999849 |
| 路口后 | 2.967855 | 0.437971 | -0.004938 | 0.999988 |

路口后实到位置为 `(2.950, 0.450)`，与目标平面距离约 0.022 m。停止线前点仍要在真实相机视角下确认：车体不能压住斑马线，完整灯组必须落入检测区域。

Gazebo、导航和红绿灯节点运行后，另开终端执行：

```bash
source ~/smart_ws/devel/setup.bash
roslaunch robot_navigation single_intersection_mission.launch
```

启动前应把车辆放在停止线点之前，并确认 `/move_base/status`、`/image_raw` 和 `/traffic_light/check` 正常。任务节点会实际驱动车辆；按 `Ctrl+C` 会取消目标并停车。

仓库内 ROS 包位于根目录；文档里的 Windows 路径记录原开发环境，使用时按本机目录替换。本地测试目录为 `tools/traffic_light_local`。
