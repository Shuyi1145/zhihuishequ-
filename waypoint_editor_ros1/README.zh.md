[中文](README.zh.md) | [English](README.md)

# Waypoint Editor for RViz（ROS 1）

该包为 ROS 1 导航系统提供了一个 RViz 面板和交互式路点编辑工具，用于在 RViz 中直接编辑、保存、加载并执行路点路径。

## 功能概览

- 在 RViz 中直接添加路点，并设置位置与朝向。
- 通过交互式 Marker 拖动已有路点、旋转朝向。
- 支持删除、重排路点编号，并为每个路点设置功能命令字符串。
- 支持将路点保存/加载为 `CSV`、项目 `YAML` 和导航 `*.nav.yaml`。
- 通过 `move_base` 启动、暂停、恢复和停止路点导航。
- 可在面板中加载 2D 地图或 3D `PCD` 点云。

## 编译

```bash
cd ~/catkin_ws
catkin_make --pkg waypoint_editor
source devel/setup.bash
```

## 启动

```bash
roslaunch waypoint_editor waypoint_editor.launch
```

常用启动参数如下：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `map_yaml` | `$(find robot_navigation)/map/map.yaml` | `map_server` 启动时加载的初始 2D 地图 YAML。 |
| `rviz_config` | `$(find waypoint_editor)/rviz/rviz_waypoint_editor.rviz` | 已经包含自定义面板的 RViz 配置文件。 |
| `open_rviz` | `true` | 是否自动启动 RViz。 |
| `start_map_server` | `true` | 是否启动 `map_server`，同时决定面板里的 `Load 2D Map` 是否可用。 |

如果外部 launch 已经占用了 `/map_server`，建议使用 `start_map_server:=false` 启动。本项目面板仍然可用，但 `Load 2D Map` 会被有意禁用，避免抢占地图服务。

## RViz 中的使用方法

### 1. 启动并确认插件已加载

自带的 RViz 配置已经包含 `waypoint_editor/WaypointEditorPanel` 面板，以及 `/map`、`/waypoint_editor/pcd_map`、路径等显示项。

如果你手动打开 RViz 后没有看到该面板，可以手动添加：

1. 打开 `Panels -> Add New Panel...`
2. 选择 `waypoint_editor/WaypointEditorPanel`
3. 确认顶部工具栏里存在名为 `Add Waypoint` 的工具

### 2. 加载地图

- 点击 `Load 2D Map`，选择一个 `.yaml` 地图文件，替换当前 `map_server` 地图。
- 点击 `Load 3D PCD`，选择一个 `.pcd` 文件并发布到 `/waypoint_editor/pcd_map`。

### 3. 在 RViz 中添加和编辑路点

1. 在 RViz 顶部工具栏选择 `Add Waypoint`。
2. 在地图上点击并拖动，设置新路点的位置和朝向。
3. 拖动绿色控制点可在 XY 平面移动路点。
4. 拖动红色旋转控制柄可修改航向角。
5. 右键单个路点可打开菜单：
   - `Delete Waypoint`
   - `Change Waypoint ID`
   - `Edit Function Command`

每个路点上方会显示 `ID:<n>`。如果该路点设置了功能命令，该命令会显示在下一行，并在保存到文件时一并写出。

### 4. 保存、加载与执行路径

- 点击 `Load WPs` 可从 `CSV`、`YAML` 或 `*.nav.yaml` 导入路点。
- 点击 `Save WPs` 可导出当前路点序列。
- 编辑阶段可以使用 `Undo`、`Redo`、`Clear All`。
- 导航阶段使用 `Start Nav`、`Pause`、`Resume`、`Stop` 控制 `move_base` 执行。

## 面板说明

### Editor 分组

| 界面项 | 含义 | 取值 / 行为 |
| --- | --- | --- |
| `Status` | 当前面板的最新状态消息。 | 例如：`Start Waypoint Editor!`、`Loaded 12 waypoints`、`Failed to load 2D Map`、`Service not available: /waypoint_editor/start_navigation`。 |
| `Waypoints` | 当前路点数量。 | 整数 `0..N`，来自 `/waypoint_editor/waypoint_count`。 |
| `Edit State` | 当前是否允许编辑。 | 空列表时为 `Add or load waypoints`，已有路点时为 `Ready for edits`，导航进行中或暂停中时为 `Locked during active navigation`。 |

### Navigation 分组

| 界面项 | 含义 | 取值 / 行为 |
| --- | --- | --- |
| `Navigation` | 导航主状态。 | `Inactive`（`IDLE`）、`Active`（`FOLLOWING`）、`Paused`、`Finished`、`Error`、`Unknown`。 |
| `Localization` | 当前是否可以连接导航后端。 | 当 `move_base` action server 已连接时显示 `Proxy ready (move_base)`，否则显示 `Proxy inactive (move_base)`。 |
| `Feedback` | 导航执行过程中的动作层反馈。 | 常见值包括：`Pending`、`Starting`、`Resuming`、`Active`、`Paused`、`Succeeded`、`Aborted`、`Rejected`、`Lost`、`Move Base Unavailable`。 |
| `Target waypoint` | 当前正在执行的目标路点。 | 导航中显示 `WP i / N`，完成后显示 `Done`，其他情况下显示 `Inactive` 或 `Unknown`。 |
| `Poses remaining` | 从当前目标开始还剩多少个路点。 | 整数计数；没有路点或执行完成时显示 `0`。 |
| `Distance remaining` | 剩余路径距离的下界估计。 | 导航中显示 `>= x.xxx m`，完成时显示 `0.000 m`，其他情况下显示 `Inactive` 或 `Unknown`。 |
| `Time taken` | 当前导航已经消耗的时间。 | 格式为 `MM:SS` 或 `H:MM:SS`；也可能显示 `Inactive` 或 `Unknown`。 |
| `Start Nav` | 启动整条路点路径导航。 | 仅在 `move_base` 已就绪、至少存在一个路点，并且状态为 `IDLE`、`FINISHED` 或 `ERROR` 时可用。 |
| `Pause` | 暂停当前导航。 | 仅在状态为 `FOLLOWING` 时可用。 |
| `Resume` | 恢复已暂停的导航。 | 仅在状态为 `PAUSED`、`move_base` 就绪且当前目标索引有效时可用。 |
| `Stop` | 停止当前导航。 | 仅在导航进行中或暂停中可用。 |

### Map 分组

| 界面项 | 含义 | 取值 / 行为 |
| --- | --- | --- |
| `Load 2D Map` | 选择新的 YAML 地图并替换当前 `map_server`。 | 仅在面板拥有 `map_server` 且没有激活导航时可用。若使用 `start_map_server:=false` 启动，则该按钮会被禁用。 |
| `Load 3D PCD` | 选择 `.pcd` 文件并发布到 `/waypoint_editor/pcd_map`。 | 仅在未执行导航时可用，内部通过 `pcl_ros/pcd_to_pointcloud` 发布，并使用 `map` 坐标系。 |

### Waypoints 分组

| 界面项 | 含义 | 取值 / 行为 |
| --- | --- | --- |
| `Undo` | 撤销最近一次路点编辑。 | 仅在存在撤销历史且未执行导航时可用。 |
| `Redo` | 重做刚刚撤销的编辑。 | 仅在存在重做历史且未执行导航时可用。 |
| `Clear All` | 清空所有路点。 | 仅在至少存在一个路点且未执行导航时可用。 |
| `Load WPs` | 打开文件对话框并导入路点。 | 支持 `*.csv`、`*.yaml`、`*.nav.yaml`，导航进行中会被禁用。 |
| `Save WPs` | 导出当前路点。 | 支持写出 `CSV`、项目 `YAML`、导航 `*.nav.yaml`；无路点或导航进行中时会被禁用。 |

## 路点文件格式

| 格式 | 扩展名 | 内容 |
| --- | --- | --- |
| CSV | `.csv` | 包含 `id`、位置（`x y z`）、姿态四元数（`x y z w`）以及可选命令列。 |
| 项目 YAML | `.yaml` | 每个路点包含 `id`、`frame_id`、完整 `pose` 和可选 `command`。 |
| 导航 YAML | `.nav.yaml` | 顶层包含 `frame_id`，下方 `waypoints` 列表中保存 `position`、`orientation` 和可选 `command`。 |

## 交互与行为说明

- 新建路点默认使用 `map` 坐标系。
- 加载的路点会保留原始 `frame_id`；如果文件中没有，则回退到 `map`。
- 对路点执行删除、改序、清空等编辑时，当前导航会先被停止，以保持状态一致。
- 导航激活后，编辑和地图加载按钮会被锁定，这是设计上的保护机制。
- 路点连线预览发布到 `/waypoint_editor/waypoints_path`。
- 3D 点云地图预览发布到 `/waypoint_editor/pcd_map`。

## 推荐使用流程

1. 启动本项目和 RViz。
2. 根据需要加载 2D 地图或 3D PCD。
3. 在 RViz 中选择 `Add Waypoint` 并依次放置路点。
4. 用交互式 Marker 调整位置和朝向。
5. 如有需要，右键路点修改编号或填写功能命令。
6. 将路径保存为 `CSV`、`.yaml` 或 `.nav.yaml`。
7. 点击 `Start Nav`，通过 `move_base` 执行整条路径。
8. 根据情况使用 `Pause`、`Resume`、`Stop`。
