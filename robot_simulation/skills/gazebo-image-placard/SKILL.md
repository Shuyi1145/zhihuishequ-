---
name: gazebo-image-placard
description: 用户用自然语言要求“根据图片生成指定长宽高的 Gazebo 模型”时，自动生成带双面贴图视觉和盒体碰撞的 Gazebo Classic 静态立牌。适用于标牌、人偶剪影等平面物料，不适用于完整立体物体建模。
---

# Gazebo 图片立牌

用户只需用 prompt 给出图片和实际尺寸；不要要求用户手动运行 Python。使用本 skill 时：

1. 定位用户给出的图片。相对路径从当前项目解析；路径有歧义时再询问。
2. 将尺寸换算为米。用户说“长宽高”时依次对应 X、Y、Z；立牌语境中的水平宽度对应 X，厚度或深度对应 Y，高度对应 Z。模型原点在底边中心。不要从图片像素比例推断物理尺寸；缺少必要尺寸时只询问缺失值。
3. 用户未指定模型名时使用脚本自动生成的名称。未指定输出目录时，优先使用当前项目的 `src/robot_simulation/models`，否则使用当前项目的 `models`。在输出目录已包含同名模型时，只有用户明确要求更新或覆盖才使用 `--overwrite`。
4. 由你执行下面的生成脚本，检查输出目录中的视觉、碰撞、纹理引用与尺寸。可用时运行 `gz sdf -p <模型目录>/model.sdf` 检查 Gazebo 解析。向用户提供生成结果和模型路径，不把命令作为待办交给用户。

使用本 skill 同目录上两级的 `scripts/generate_gazebo_placard.py`。它接收 PNG、JPEG 等 Pillow 可读取的图片，生成 `model.config`、`model.sdf`、带 UV 的双面 DAE、PNG 纹理和透明材质。原图片有 alpha 通道时保留透明区域；没有时生成不透明面板。它生成贴图立牌，不会从单张图片重建真实 3D 几何。

将 prompt 中的值代入脚本参数，例如“根据 `/path/to/image.png`，生成长 5 cm、宽 5 mm、高 15 cm 的 Gazebo 模型”：

```bash
python3 src/robot_simulation/scripts/generate_gazebo_placard.py \
  --image /path/to/image.png \
  --width 0.05 --thickness 0.005 --height 0.15 \
  --name example_sign \
  --output-dir /path/to/gazebo/models
```

`--name` 可省略，脚本会从图片名生成安全的模型名。若任务是本项目的整套人员素材，运行 `scripts/generate_person_models.py`；该入口会调用同一个单图生成器，当前默认尺寸为 `0.05 × 0.005 × 0.15 m`，更新既有模型使用 `--force`。

如果已有 Gazebo 实例加载了旧模型，需要重新插入或重启场景才能看到更新。Gazebo 模型路径需包含输出父目录，world 中通过 `model://<模型名>` 引用。报告未完成的渲染验证，不将 SDF 解析等同于 GUI 渲染确认。
