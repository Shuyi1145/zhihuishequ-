# 红绿灯 ROS 1 Noetic 节点

这是本次新建的独立 catkin 包。复用本地 `traffic_light_cv.py` 的检测算法；原本地图片脚本保留不变。ROS 节点持续订阅 `sensor_msgs/Image`，不再读取本地截图，也不会在绿灯后退出或控制底盘。示例客户端查询服务、等待绿灯后结束自身。

## 安装到虚拟机

将整个 `traffic_light_detector` 文件夹复制到 `/home/GGB/smart_ws/src/`。最终应存在 `/home/GGB/smart_ws/src/traffic_light_detector/package.xml`，不要多套同名目录。

在 Ubuntu 终端执行：

```bash
source /opt/ros/noetic/setup.bash
sudo apt install ros-noetic-cv-bridge python3-opencv python3-numpy
cd ~/smart_ws
catkin_make
source devel/setup.bash
```

若构建失败，先处理报错，不继续运行。`catkin_install_python` 负责生成可执行脚本入口；源码编辑器如改变首行，应保持 Python 3 和 LF 换行。

## 启动和调用

保持 Gazebo 小车与相机运行。已检查的机器人相机配置写的是 `image_raw`，但以虚拟机实际话题为准。

```bash
source ~/smart_ws/devel/setup.bash
roslaunch traffic_light_detector traffic_light.launch image_topic:=/image_raw
```

新终端进行一次查询：

```bash
source ~/smart_ws/devel/setup.bash
rosservice call /traffic_light/check "{}"
```

本包新定义的接口：

| 名称 | 类型 | 语义 |
|---|---|---|
| `/traffic_light/check` | `std_srvs/Trigger` 服务 | `success=true` 表示绿灯已确认且结果新鲜；false 表示不可通行；message 是 JSON，含 state、allowed、reason |
| `/traffic_light/reset` | `std_srvs/Trigger` 服务 | 清空确认状态，开始新观察任务；success 表示重置是否成功 |
| `/traffic_light/state` | `std_msgs/String` 话题 | JSON 状态，供调试显示 |
| `/traffic_light/image` | `sensor_msgs/Image` 话题 | 带圈选和指标的图像，保留原相机时间戳 |

`check` 是立即查询，不阻塞等待绿灯。它的 success 特意定义为是否允许通行，不代表服务调用有没有执行。红灯、黄灯、未知或绿灯尚未连续确认都会返回 false。

RViz 添加 Image，选择 `/traffic_light/image` 查看结果。可用 `rostopic echo /traffic_light/state` 看状态。

运行等待示例：

```bash
source ~/smart_ws/devel/setup.bash
rosrun traffic_light_detector wait_for_green.py _timeout:=120
```

客户端启动先 reset，然后反复查询，打印“当前是：…灯”“通行：不可通行/可通行”“等待中.....。”，允许后打印“通过中.....。”并退出。退出码 0 表示已确认绿灯，1 表示等待失败；示例不会移动小车。识别服务节点仍持续运行。

## 接入 mission.py 的位置

接入顺序：导航到停止线前的安全观察点 → 确认停稳并阻止导航继续发运动指令 → 调用 reset → 周期调用 check → success 为 true 且解析状态为 green 才允许发送下一个目标。异常与超时保持停车，并将任务标为失败。

可以复用等待示例中的逻辑，创建 `rospy.ServiceProxy('/traffic_light/check', Trigger)`。服务的 message 是 JSON，不是原 `robot_navigation/detect` 的字符串结果协议，不能把两个服务类型混用。服务不可用时不要继续下一导航点。ROS 服务调用自身是同步的，示例的 120 秒是两次调用之间检查的任务期限，不是网络调用的硬超时。

此节点不发布 `/cmd_vel`，停车执行、速度互锁、路线任务管理由任务节点负责。本版只处理单个当前观察视角，不能让多个路口任务同时调用 reset，也不能将远处其他灯组当成当前路口。起步后仍需结合停止线位置处理变灯；一次查询通过不是全程绿灯保证。

## 新帧和参数

默认 5 Hz 检测，仅处理最新图像；连续至少 3 个不同时间戳的图像、持续至少 0.5 秒支持绿灯，才允许通行。任一红灯、黄灯、未知判定立即清除绿灯累计。墙钟接收时间与 ROS 图像时间共同检查新鲜度，默认超过 1 秒失效，Gazebo 暂停或相机断流也不会无限保留绿灯。没有有效 header 时间戳的图像被拒绝。仿真重置需重新发起观察任务。

`launch/traffic_light.launch` 中可修改 image_timeout、green_hold、green_min_frames、detection_rate。CV 参数来自 `src/traffic_light_detector/detector.py` 的 Settings，也可用同名节点私有参数配置。

默认搜索图像上方 45%，半径范围和亮度阈值沿用本地截图。机器人原始相机图像的裁剪、长宽比、观察距离变化后必须重新验证。可临时使用：

```bash
roslaunch traffic_light_detector traffic_light.launch image_topic:=/image_raw roi_bottom:=1.0
```

## 验证边界

已在 Windows 检查 Python 语法、XML、四项纯 Python 时序门控测试，以及所提供绿灯截图上的检测。2026-09-26 用户已在 Ubuntu/ROS 环境启动节点，日志依次出现 unknown/False、red/False、green/False、green/True，证明相机检测及绿灯延时确认已初步运行。green/False 到 green/True 的墙钟间隔约 0.69 秒。

上述 ROS 运行证据来自用户日志；本机没有 ROS Noetic 运行时，未直接执行 catkin 构建或验证服务调用。黄灯、服务通信、相机断流、暂停仿真、重新观察及两处路口的视角变化仍待实测；未完成这些测试前不能把它视为已验收的车辆通行控制。当前阶段完成的是识别，不包括导航停车和底盘运动互锁。

package.xml 中维护者信息是占位元数据，正式交付时替换为项目维护者信息。
