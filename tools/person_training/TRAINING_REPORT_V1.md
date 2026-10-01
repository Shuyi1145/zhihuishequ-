# 人物识别模型 v1 训练报告

## 训练环境

- Conda 环境：`C:\ProgramData\miniconda3\envs\y8`
- Python：3.10.19
- PyTorch：2.9.1+cu128
- Ultralytics：8.3.236
- GPU：NVIDIA GeForce RTX 4060 Laptop GPU（8 GB）

## 数据集检查

- 训练集：13 张图片，78 个框
  - `community`：65
  - `non-community`：13
- 验证集：6 张图片，26 个框
  - `community`：23
  - `non-community`：3
- 图片和标签一一对应。
- 标签均为合法的 YOLO HBB 格式。
- 没有发现类别反标、坐标越界或明显漏标。

## 基线模型

模型：`codev3-github/yolo_ros/weights/best.pt`

| 类别 | mAP50 | mAP50-95 |
|---|---:|---:|
| community | 0.587 | 0.257 |
| non-community | 0.139 | 0.056 |
| 整体 | 0.363 | 0.156 |

## 微调结果

训练从现有 `best.pt` 开始，计划 60 轮，在第 38 轮早停，最佳权重来自第 23 轮。

模型位置：

`D:\GPTProject\ros2\zhihuishequ\person_training\runs\person_v1\weights\best.pt`

| 类别 | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|
| community | 0.950 | 0.870 | 0.892 | 0.582 |
| non-community | 0.926 | 1.000 | 0.995 | 0.641 |
| 整体 | 0.938 | 0.935 | 0.944 | 0.612 |

## 已发现风险

1. 验证集只有 3 个 `non-community` 实例，指标可信度有限。
2. 所有图片来自相近的 Gazebo 场景，验证结果不能代表新视角和新距离。
3. `scene_0013.jpg` 中，画面最左侧被裁切的蓝色小车被高置信度误判成 `community`。
4. 当前数据只覆盖一种蒙面 `non-community` 人物，无法证明模型能识别另一种非社区人物。
5. ROS 节点直接统计检测框数量；任何重复框或误检都会造成错误人数。

## 下一轮数据要求

- 先拍摄 10～20 张全新测试图片，不能加入训练集，用于检验泛化。
- 采集 5～10 张没有人物但包含车辆、道路、红绿灯和地图边缘的图片，作为空标签负样本加入训练集。
- 如果比赛会出现另一种非社区人物，必须补充该人物的训练和验证图片。
- 新模型通过独立测试后，再替换 ROS 中的正式权重。
