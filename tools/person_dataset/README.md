# 人员识别数据集制作

本目录用于把 Gazebo 相机采集的无人物背景与 `robot_simulation/models` 中的透明人物素材合成为 YOLO 检测数据集。类别保持与现有 `best.pt` 一致：`0=community`、`1=non-community`。

背景必须来自 `/image_raw` 且画面中没有人物。脚本从 `materials/textures` 读取 16 个社区人员和 2 个非社区人员透明 PNG，随机调整人物的大小、位置、角度、亮度与模糊程度，同时自动写出 YOLO 边界框标签。训练与验证背景按文件划分；至少准备两个背景，建议包含多个真实停车位置和朝向。

## 采集无人物背景

先在 Gazebo 中临时删除或移走所有人物，然后创建目录：

```bash
mkdir -p "$HOME/person_dataset/backgrounds"
```

启动按需保存节点：

```bash
rosrun image_view image_saver \
  __name:=background_saver \
  image:=/image_raw \
  _save_all_image:=false \
  _filename_format:="$HOME/person_dataset/backgrounds/background_%04i.jpg"
```

另开终端，每改变一次车辆位置、距离或朝向后保存一张：

```bash
rosservice call /background_saver/save "{}"
```

如果服务名不同，用 `rosservice list | grep save` 查出实际名称。不要靠车辆静止时连续保存大量几乎相同的帧；背景多样性比帧数更重要。

## 采集真实含人物场景

恢复人物模型后，把真实场景放到独立目录，不能混入无人物背景：

```bash
mkdir -p "$HOME/person_dataset/real_scenes"

rosrun image_view image_saver \
  __name:=person_scene_saver \
  image:=/image_raw \
  _save_all_image:=false \
  _filename_format:="$HOME/person_dataset/real_scenes/scene_%04i.jpg"
```

保存一帧：

```bash
rosservice call /person_scene_saver/save "{}"
```

这些真实场景需要人工标注全部可见人物；任何没有标注的人都会被训练过程当作背景。

## 自动合成

```bash
python3 tools/person_dataset/generate_synthetic_dataset.py \
  --models-dir "$(rospack find robot_simulation)/models" \
  --backgrounds-dir "$HOME/person_dataset/backgrounds" \
  --output-dir "$HOME/person_dataset/generated" \
  --train-count 400 \
  --val-count 100
```

生成结果包含 `images/train`、`images/val`、`labels/train`、`labels/val` 和 `persons.yaml`。检查若干图片及同名标签后，再用现有 `best.pt` 微调。真实的含人物 Gazebo 图片不能作为无人物背景；它们应单独人工标注后加入数据集。

## 使用现有模型微调

```bash
export PYTHONPATH="$(rospack find yolo_ros)/src/ultralytics${PYTHONPATH:+:$PYTHONPATH}"

python3 - <<'PY'
from pathlib import Path
from ultralytics import YOLO

home = Path.home()
model = YOLO(str(home / "smart_ws/src/smart_community/yolo_ros/weights/best.pt"))
model.train(
    data=str(home / "person_dataset/generated/persons.yaml"),
    epochs=50,
    imgsz=640,
    batch=8,
    device="cpu",
    project=str(home / "person_training"),
    name="finetune_v1",
)
PY
```

有可用的 NVIDIA CUDA 环境时才把 `device="cpu"` 改为 `device=0`。先保留原 `best.pt`；训练输出通常位于 `$HOME/person_training/finetune_v1/weights/best.pt`，必须通过独立验证集和两批 Gazebo 场景后才能替换运行模型。
