# zhihuishequ-
智慧社区ROS1参考代码

## 红绿灯识别（2026-09-26）

- [ROS Noetic 节点与调用说明](traffic_light_detector/README.md)：相机图像检测、绿灯连续确认、check/reset 服务。
- [本地图片测试](tools/traffic_light_local/使用说明.md)：直接读取图片验证传统 CV 检测。
- [复赛完赛规划与阶段进度](docs/智慧社区复赛完赛规划.md)。

识别节点已在用户仿真环境中输出红灯不可通行、绿灯待确认及绿灯可通行。导航停车、运动互锁以及黄灯/断流/暂停等场景仍需联调验证。

仓库内 ROS 包位于根目录；文档里的 Windows 路径记录原开发环境，使用时按本机目录替换。本地测试目录为 `tools/traffic_light_local`。
