#!/usr/bin/env bash
# Start the confirmed Gazebo scene and wait for each ROS dependency before driving.
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
workspace_dir="$(cd -- "$repo_dir/../.." && pwd)"
setup_file="$workspace_dir/devel/setup.bash"

if [[ ! -f "$setup_file" ]]; then
  echo "找不到 $setup_file；请先在工作空间完成 catkin_make。" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$setup_file"
set -u

for package in robot_simulation robot_navigation traffic_light_detector yolo_ros; do
  if ! rospack find "$package" >/dev/null 2>&1; then
    echo "找不到 ROS 包 $package；请检查工作空间构建与环境加载。" >&2
    exit 1
  fi
done
for package in robot_simulation robot_navigation; do
  actual_dir="$(cd -- "$(rospack find "$package")" && pwd)"
  if [[ "$actual_dir" != "$repo_dir/$package" ]]; then
    echo "$package 实际来自 $actual_dir，预期为 $repo_dir/$package。" >&2
    echo "请检查工作空间中是否有同名 ROS 包。" >&2
    exit 1
  fi
done

world_file="$repo_dir/robot_simulation/world/competition.world"
map_yaml="$repo_dir/robot_navigation/map/map_portable.yaml"
person_weights="$repo_dir/yolo_ros/weights/person_v4.pt"
if [[ ! -f "$world_file" || ! -f "$map_yaml" || ! -f "$person_weights" ]]; then
  echo "场景、地图或人物 v4 权重缺失；请先更新工程。" >&2
  exit 1
fi
if ! grep -q 'traffic_light' "$world_file" || ! grep -q 'person_community' "$world_file"; then
  echo "当前 competition.world 没有两处路口任务所需的灯和人物立牌。" >&2
  echo "请保留你在虚拟机中已调好的比赛场景，不要启动仓库中的旧场景。" >&2
  exit 1
fi
if [[ ! -f "$repo_dir/robot_simulation/models/person_community_01/model.sdf" ]]; then
  echo "当前工程缺少人物立牌模型文件；请检查虚拟机中的比赛场景资源。" >&2
  exit 1
fi
if grep -q 'libtraffic_light_controller.so' "$world_file" &&
   [[ ! -f "$workspace_dir/devel/lib/libtraffic_light_controller.so" ]]; then
  echo "提醒：工作空间中未找到交通灯插件，请确认虚拟机使用的灯能正常切换。" >&2
fi

python3 - "$map_yaml" <<'PY'
from pathlib import Path
import sys

yaml_path = Path(sys.argv[1])
line = next((item for item in yaml_path.read_text(encoding='utf-8').splitlines()
             if item.startswith('image:')), None)
if line is None:
    sys.exit('地图 YAML 缺少 image 字段。')
image_path = Path(line.partition(':')[2].strip().strip('"\''))
if not image_path.is_absolute():
    image_path = yaml_path.parent / image_path
if not image_path.is_file():
    sys.exit('地图图像不存在：{}'.format(image_path))
PY

if rosnode list 2>/dev/null | grep -Eq '^/(gazebo|gazebo_gui|move_base|traffic_light|detect_ros|two_intersection_mission)$'; then
  echo "检测到本任务的 ROS 节点已在运行；请先关闭旧的 Gazebo/导航/识别终端。" >&2
  exit 1
fi

export GAZEBO_MODEL_PATH="$repo_dir/robot_simulation/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$workspace_dir/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"

launch_pids=()
cleanup() {
  for ((index=${#launch_pids[@]}-1; index>=0; index--)); do
    kill "${launch_pids[index]}" 2>/dev/null || true
  done
  for pid in "${launch_pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT

start_launch() {
  echo "启动：roslaunch $*"
  roslaunch --screen "$@" &
  launch_pids+=("$!")
}

check_launches() {
  for pid in "${launch_pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "一个启动进程提前退出，请查看上方错误日志。" >&2
      exit 1
    fi
  done
}

wait_topic() {
  local topic="$1" deadline=$((SECONDS + $2))
  echo "等待话题 $topic ..."
  while (( SECONDS < deadline )); do
    check_launches
    if timeout 4 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
      return 0
    fi
  done
  echo "等待 $topic 超时。" >&2
  exit 1
}

wait_service() {
  local service="$1" deadline=$((SECONDS + $2))
  echo "等待服务 $service ..."
  while (( SECONDS < deadline )); do
    check_launches
    if rosservice list 2>/dev/null | grep -Fxq "$service"; then
      return 0
    fi
    sleep 1
  done
  echo "等待 $service 超时。" >&2
  exit 1
}

echo '第 1 步：Gazebo 与小车'
start_launch robot_simulation simulation_robot.launch
wait_topic /image_raw/header 180

echo '第 2 步：导航与 RViz'
start_launch robot_navigation navigation.launch simulation:=true "map_file:=$map_yaml"
wait_topic /map/info 90
wait_topic /move_base/status 120

python3 - <<'PY'
import time
import rospy
import tf

rospy.init_node('mission_startup_tf_check', anonymous=True, disable_signals=True)
listener = tf.TransformListener()
deadline = time.monotonic() + 90.0
while time.monotonic() < deadline and not rospy.is_shutdown():
    try:
        listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
        print('地图到车体的坐标变换已就绪。')
        break
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        time.sleep(0.5)
else:
    raise SystemExit('等待 map -> base_footprint 坐标变换超时。')
PY

echo '第 3 步：红绿灯识别'
start_launch traffic_light_detector traffic_light.launch image_topic:=/image_raw
wait_service /traffic_light/check 90
wait_service /traffic_light/reset 90

echo '第 4 步：人物 v4 识别'
start_launch yolo_ros yolo.launch "weights:=$person_weights" input_image_topic:=/image_raw
wait_service /recognize_person 180

echo '第 5 步：两路口任务开始；按 Ctrl+C 可关闭整套仿真。'
roslaunch --screen robot_navigation two_intersection_mission.launch
echo '任务进程已结束。Gazebo 保持打开，按 Ctrl+C 关闭整套仿真。'
while true; do sleep 1; done
