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

`single_intersection_mission.py` 可在第一处路口后依次到 A、B 街区观察点，各停稳并调用一次现有的 `/recognize_person`（`robot_navigation/detect`）服务，再经 B 街区后的拐角导航到第二处红绿灯停止线前，重置灯色确认并等待稳定绿灯。到第二处灯后本阶段仍停车，不越过该路口。原来的单路口启动方式不变；只有设置 `two_area_enabled:=true` 才执行扩展流程。该阶段目前是代码接入，尚未在虚拟机完成整段实测。

用户提供了以下 `map → base_footprint` 实到位姿，已作为 launch 默认值。它们是车体中心位姿，不自动证明导航目标可达，也不证明车身在停止线前。

| 点位 | x (m) | y (m) | qz | qw |
|---|---:|---:|---:|---:|
| A 街区观察点 | 1.650 | 1.100 | 1.000 | 0.000 |
| B 街区观察点 | 2.550 | 3.050 | 1.000 | -0.009 |
| B 街区后拐角 | 2.450 | 2.100 | 1.000 | 0.019 |
| 第二处斑马线附近 | 1.700 | 2.150 | 1.000 | 0.026 |

**启用扩展任务前**，在 RViz 中依次验证第一处路口后目标点 → A → B → 拐角 → 第二处点的路径可达，确认每次停稳后相机只覆盖对应街区，并确认第二处点的**整个车身**位于停止线前、摄像头能看见第二组灯。B → 拐角的位移主要朝地图 y 负方向，但给出的拐角朝向约 178°（朝 x 负方向）；`move_base` 可能在拐角停车调整朝向后才继续，应检查实际轨迹是否符合比赛路线。若第二处点落在斑马线上，应重新停车测量车身安全位置，再覆盖 `second_stop_*` 参数。圆整后的四元数会在节点内归一化。

验证通过后启动扩展流程：

```bash
roslaunch robot_navigation single_intersection_mission.launch two_area_enabled:=true
```

启动前确认 `/recognize_person`、`/image_raw`、导航和红绿灯服务均在运行。A/B 分别以 `detect_flag=1/2` 触发同一个 YOLO 模型；该标志本身不会切换识别类别或限定视野。观察方向必须让每次画面只覆盖对应街区，否则同一立牌可能被两次计数。程序分别记录 A/B 人数，在视野不重叠尚未确认前不直接相加。第二处灯复用同一个红绿灯检测服务，必须确认检测到的是**第二组**灯，而非第一处灯或场景中其他亮点。当前位置不合适时，可通过 `second_stop_x:=...` 等同名参数覆盖默认值。

仓库内 ROS 包位于根目录；文档里的 Windows 路径记录原开发环境，使用时按本机目录替换。本地测试目录为 `tools/traffic_light_local`。

## 新路径：通过第二处路口

`robot_navigation/launch/two_intersection_mission.launch` 使用 `code/路径点.md` 中新测量的地图位姿，依次执行：第一处停车等绿灯、过路口、路口后两个拐点和街区中间点、A 街区识别、B 街区识别、第二处拐点、第二处停车等绿灯、越过第二处路口后结束。A/B 各调用一次 `/recognize_person`；两处灯复用 `/traffic_light/reset` 和 `/traffic_light/check`，保持原单路口的确认逻辑。车牌和返程尚不在此阶段。原来的 `single_intersection_mission.launch` 不受影响。

**目前新路径故意不能直接启动。** `路径点.md` 中“转向90°后”记录为 `(x=0.400, y=0.500, qz=-0.009, qw=1.000)`；与上一点 `(3.700, 0.400)` 相距约 3.30 m，且朝向只由约 2° 变为 -1°，并非 90°。对应的四个启动参数默认是 `UNSET`，任务会在发车前报错。请在同一次稳定定位的仿真运行中重新测得该点的 `map -> base_footprint` 位姿，并先在 RViz 用 `2D Nav Goal` 单独验证整个路线、停车视角和车身是否位于停止线前。手动拖动车辆后的 TF 可能仍反映旧定位，单看 Gazebo 位置无法证明地图坐标正确。

确认并填入真实值后，在 Gazebo、`navigation.launch simulation:=true`、红绿灯节点和 YOLO/人物识别服务都运行的情况下另开终端：

```bash
source ~/smart_ws/devel/setup.bash
roslaunch robot_navigation two_intersection_mission.launch \
  turn_after_first_x:=<实测x> turn_after_first_y:=<实测y> \
  turn_after_first_qz:=<实测qz> turn_after_first_qw:=<实测qw>
```

`<实测...>` 需替换成数字，不要原样复制尖括号。已在代码中设置的其余新路线位姿均来自用户文档，但尚未经整段自主导航实测；尤其第二处路口后目标朝向约 -171°，车辆到点时可能转身，先确认比赛路线是否需要这个朝向。任一导航或识别失败会终止任务，不跳过该阶段。
