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

## A/B 街区识别与第二处红绿灯（2026-10-01）

`single_intersection_mission.py` 可在第一处路口后依次到 A、B 街区观察点，各停稳并调用一次现有的 `/recognize_person`（`robot_navigation/detect`）服务，然后导航到第二处红绿灯停止线前，重置灯色确认并等待稳定绿灯。到第二处灯后本阶段仍停车，不越过该路口。原来的单路口启动方式不变；只有设置 `two_area_enabled:=true` 才执行扩展流程。该阶段目前是代码接入，尚未在虚拟机完成整段实测。

三个新点位（A、B、第二处停止线前）必须在当前地图中实测。如果车已停在合适位置，运行 `rosrun tf tf_echo map base_footprint`，读取当前车体在 `map` 下的 Translation `x/y` 与 Rotation Quaternion `z/w`（按 `Ctrl+C` 停止输出）。本项目代价地图使用 `base_footprint` 作为车体坐标系。也可在终端先运行 `rostopic echo -n 1 /move_base_simple/goal`，再用 RViz 的 **2D Nav Goal** 选可达的点，记录 `header.frame_id=map`、`position.x/y` 和 `orientation.z/w`；点击会同时发送导航目标。只查看平面位置可用 **Publish Point** 和 `rostopic echo -n 1 /clicked_point`，但这不提供朝向。

把每处实测的四个值代入以下命令；大写标记均为占位符，不是场地坐标：

```bash
roslaunch robot_navigation single_intersection_mission.launch \
  two_area_enabled:=true \
  area_a_x:=A_X area_a_y:=A_Y area_a_qz:=A_QZ area_a_qw:=A_QW \
  area_b_x:=B_X area_b_y:=B_Y area_b_qz:=B_QZ area_b_qw:=B_QW \
  second_stop_x:=SECOND_X second_stop_y:=SECOND_Y \
  second_stop_qz:=SECOND_QZ second_stop_qw:=SECOND_QW
```

启动前确认 `/recognize_person`、`/image_raw`、导航和红绿灯服务均在运行。A/B 分别以 `detect_flag=1/2` 触发同一个 YOLO 模型；该标志本身不会切换识别类别或限定视野。观察方向必须让每次画面只覆盖对应街区，否则同一立牌可能被两次计数。程序分别记录 A/B 人数，在视野不重叠尚未确认前不直接相加。未填真实坐标会在发车前报错；不能用旧路线或示意图像素代替 `map` 坐标。

仓库内 ROS 包位于根目录；文档里的 Windows 路径记录原开发环境，使用时按本机目录替换。本地测试目录为 `tools/traffic_light_local`。
