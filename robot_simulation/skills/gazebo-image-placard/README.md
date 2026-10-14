# Gazebo Image Placard

从一张图片和实际尺寸生成可直接加载到 **Gazebo Classic** 的静态立牌模型。输出包含双面贴图视觉、透明材质和盒体碰撞，不需要 Blender。适合人偶立牌、标牌和其他平面物料；它不会把图片重建为立体物体。

## 环境要求

- Python 3.8+ 和 Pillow。
- Gazebo Classic（加载、渲染及使用模型时需要；仅生成文件时不需要）。

生成脚本位于 [`../../scripts/generate_gazebo_placard.py`](../../scripts/generate_gazebo_placard.py)，DAE 模板位于 [`../../scripts/templates/placard.dae`](../../scripts/templates/placard.dae)。**请保留仓库目录结构**；只下载本 skill 目录无法运行生成脚本。

如未安装 Pillow，可在所用 Python 环境中运行：

```bash
python3 -m pip install Pillow
```

## 安装为 Codex Skill

克隆本仓库后，在仓库根目录执行：

```bash
mkdir -p "$HOME/.codex/skills"
ln -s "$PWD/src/robot_simulation/skills/gazebo-image-placard" \
  "$HOME/.codex/skills/gazebo-image-placard"
```

重新启动 Codex 后，直接描述任务即可触发 skill，无需自己调用 Python。例如：

> 根据 `/path/to/person.png`，生成长 5 cm、宽 5 mm、高 15 cm 的 Gazebo 立牌模型。

> 用 `复赛资料/人员/社区人员/1.png` 做一个 Gazebo 模型：水平宽 5 cm、厚 5 mm、高 15 cm，保存到 `src/robot_simulation/models`。

Codex 会定位图片、换算为米、选择输出目录、调用生成脚本并检查结果。模型名和输出目录可以省略：默认从图片文件名生成模型名，并优先输出到当前项目的 `src/robot_simulation/models`。图片路径或必要尺寸不明确时，Codex 才会追问。也可以显式写 `$gazebo-image-placard`，但通常不需要。

## 尺寸约定

所有尺寸最终以米传给 Gazebo。“长宽高”依次对应 X、Y、Z：X 为立牌水平宽度，Y 为厚度，Z 为高度。例如 `5 cm × 5 mm × 15 cm` 会转换为 `0.05 × 0.005 × 0.15 m`。图片会完整映射到指定宽高的面片上，因此图片比例与物理宽高比不同时，画面会拉伸。

## 命令行生成（可选）

在仓库根目录运行：

```bash
python3 src/robot_simulation/scripts/generate_gazebo_placard.py \
  --image /path/to/person.png \
  --width 0.05 \
  --thickness 0.005 \
  --height 0.15 \
  --name person_01 \
  --output-dir src/robot_simulation/models
```

命令行参数使用 **米**。`--width` 是 X 轴水平宽度，`--thickness` 是 Y 轴厚度，`--height` 是 Z 轴高度；模型原点位于立牌底边中心。

| 参数 | 作用 |
| --- | --- |
| `--image` | 输入图片路径；支持 Pillow 可读取的 PNG、JPEG 等格式。 |
| `--width` | 立牌宽度（m），同时用于视觉面片和碰撞体。 |
| `--thickness` | 盒体碰撞厚度（m）；视觉面片仍为平面。 |
| `--height` | 立牌高度（m），同时用于视觉面片和碰撞体。 |
| `--name` | 模型名；可省略，脚本会根据图片文件名生成安全名称。 |
| `--output-dir` | 模型的父目录；省略时为当前目录下的 `models`。 |
| `--overwrite` | 更新同名模型；默认遇到同名目录会跳过。 |

PNG 的透明区域会保留。没有透明通道的图片会转换为不透明的 PNG 纹理。执行后目录结构如下：

```text
models/person_01/
├── model.config
├── model.sdf
├── meshes/
│   ├── person_01.dae
│   └── person_01.png
└── materials/
    ├── scripts/person_01.material
    └── textures/person_01.png
```

## 在 Gazebo 中使用

确保 `GAZEBO_MODEL_PATH` 包含模型的**父目录**。使用本仓库的 [`gazebo_world.launch`](../../launch/gazebo_world.launch) 时，`src/robot_simulation/models` 已加入模型搜索路径；使用其他输出目录时可手动设置：

```bash
export GAZEBO_MODEL_PATH="/absolute/path/to/models:${GAZEBO_MODEL_PATH:-}"
```

在 world 文件中通过模型名引用：

```xml
<include>
  <uri>model://person_01</uri>
  <name>person_01</name>
  <pose>1 1 0 0 0 0</pose>
</include>
```

生成后可检查 SDF 是否能被 Gazebo 解析：

```bash
gz sdf -p src/robot_simulation/models/person_01/model.sdf
```

若模型已经加载在 Gazebo 中，修改后需要重新插入模型或重启场景才能看到更新。

## 本项目人员素材批量生成

[`../../scripts/generate_person_models.py`](../../scripts/generate_person_models.py) 是针对本项目 `复赛资料/人员` 的批量入口，它调用同一个单图生成器。默认尺寸为宽 `0.05 m`、厚 `0.005 m`、高 `0.15 m`：

```bash
python3 src/robot_simulation/scripts/generate_person_models.py
```

需要更新已存在的人员模型时追加 `--force`。这个批量入口只处理固定的社区人员和非社区人员目录；其他图片请使用上面的单图命令。
