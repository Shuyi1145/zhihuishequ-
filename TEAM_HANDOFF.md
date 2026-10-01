# 智慧社区复赛队友交接

本仓库根目录是 ROS1 catkin 源码包集合。场景采用 `2026开源资料/robot_simulation/` 的完整包：`world/competition.world`、`models/`、交通灯 Gazebo 插件源码和构建配置均已并入 `robot_simulation/`。`simulation_robot.launch` 与 `gazebo_world.launch` 是当前仿真启动入口。

## 获取与编译

在 Ubuntu 20.04、ROS Noetic 的 catkin 工作空间中：

```bash
cd ~/smart_ws/src
git clone https://github.com/Shuyi1145/zhihuishequ-.git smart_community
cd ~/smart_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

已克隆的机器先在 `~/smart_ws/src/smart_community` 运行 `git status --short`，确认本地改动如何保留，再执行 `git pull --ff-only` 和 `catkin_make`。更新场景后重新编译，才能生成交通灯插件。运行依赖包括仓库外的 ROS/Gazebo 环境、Python 的 `torch`、`cv2` 等包和百度 OCR 网络服务；本仓库没有封装操作系统或 Python 环境。

## 分终端运行

每个终端先运行 `source ~/smart_ws/devel/setup.bash`，再分别执行：

```bash
# 终端 1：场景和小车
roslaunch robot_simulation simulation_robot.launch

# 终端 2：地图、定位、导航和 RViz
roslaunch robot_navigation navigation.launch simulation:=true

# 终端 3：红绿灯检测
roslaunch traffic_light_detector traffic_light.launch image_topic:=/image_raw

# 终端 4：人物识别；默认权重已设为 person_v4.pt
roslaunch yolo_ros yolo.launch input_image_topic:=/image_raw

# 终端 5：小车在第一处停止线之前、传感器和服务正常后启动完整任务
roslaunch --screen robot_navigation full_competition_mission.launch
```

完整任务入口会启动百度车牌 OCR 服务；不要再另开 `plate_recognition.launch`。任务路线配置在 `robot_navigation/config/full_route.yaml`，启动脚本在 `robot_navigation/scripts/single_intersection_mission.py`。`robot_navigation/map/map.yaml` 已使用相对图像路径，地图文件为同目录的 `map.pgm`。

## 已整合

| 内容 | 仓库位置 |
|---|---|
| 官方开源场景、模型、交通灯插件源码 | `robot_simulation/` |
| 小车模型与传感器 | `robot_description/` |
| 地图、定位、导航、两处路口和三车牌任务 | `robot_navigation/`、`jie_ware/`、`simple_local_planner/` |
| 红绿灯检测、人物识别和最终人物 v4 权重 | `traffic_light_detector/`、`yolo_ros/weights/person_v4.pt` |
| 百度 OCR 车牌识别 | `robot_navigation/scripts/detect_plate.py`、`robot_navigation/launch/plate_recognition.launch` |
| 训练脚本及 v4 结果摘要 | `tools/person_training/`；用于追溯训练，不是运行任务的必要文件 |
| 路径点与阶段记录 | `docs/` |

## 未整合与验证边界

- 原始人物训练图片、标签、历次训练权重和中间产物未并入；最终运行权重已经包含。`tools/person_training/train_person_v4.py` 保留原始训练路径约定，缺少原始数据和前序权重时不能直接复训。
- 官方技术报告模板、参考图片、虚拟机镜像、历史 Git 离线包和实验视频没有作为 ROS 源码打包。
- `robot_simulation/` 仍保留旧工程的 `ro1`、`ro2` 模型和示例 world 文件；当前 `gazebo_world.launch` 只加载 `competition.world`，这些旧文件不参与任务。
- 本次在 Windows 上完成文件和静态结构检查，无法在本机运行 ROS Noetic/Gazebo 整圈；首次在队友虚拟机运行仍应检查插件加载、车道规划、两处等灯、A/B 识别、三个车牌和终点停车。
- A/B 按观察画面分别计数，没有跨画面身份去重。百度 OCR 调用需要可用网络与凭据；不要在报告、截图或聊天中公开配置文件内容。

目录下的 `README.md`、`docs/报告资料索引.md` 有历史联调记录；涉及旧提交号和旧场景状态时，以本文件和当前源码为准。
