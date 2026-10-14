#!/bin/bash
set -euo pipefail

# ==============================
# 配置
# ==============================

# 当前视频目录
VIDEO_DIR="$(pwd)"

# 图片输出目录
IMAGE_DIR="/home/shuiy/catkin_ws/src/ultralytics/raw_data/image"

# 合并后的临时视频
MERGED_VIDEO="${VIDEO_DIR}/merged_all.mp4"

# 每秒抽取多少帧
FPS=3

# 图片起始编号
START_NUMBER=1000

# 图片编号位数
# 例如：
# DIGITS=4  -> 1000.png
# DIGITS=6  -> 001000.png
DIGITS=4

# 临时 concat 文件
LIST_FILE="$(mktemp)"

# 脚本结束时自动删除临时文件
trap 'rm -f "${LIST_FILE}"' EXIT


# ==============================
# 查找当前目录所有 mp4
# ==============================

echo "======================================"
echo "视频目录: ${VIDEO_DIR}"
echo "图片目录: ${IMAGE_DIR}"
echo "FPS: ${FPS}"
echo "起始编号: ${START_NUMBER}"
echo "编号位数: ${DIGITS}"
echo "======================================"

mapfile -t VIDEOS < <(
    find "${VIDEO_DIR}" \
        -maxdepth 1 \
        -type f \
        -iname "*.mp4" \
        ! -name "$(basename "${MERGED_VIDEO}")" \
        | sort
)

VIDEO_COUNT=${#VIDEOS[@]}

if [ "${VIDEO_COUNT}" -eq 0 ]; then
    echo "错误：当前目录没有找到 mp4 视频"
    exit 1
fi

echo
echo "找到 ${VIDEO_COUNT} 个视频："

for video in "${VIDEOS[@]}"; do
    echo "  $(basename "${video}")"
done


# ==============================
# 生成 FFmpeg concat 列表
# ==============================

for video in "${VIDEOS[@]}"; do
    printf "file '%s'\n" "${video}" >> "${LIST_FILE}"
done


# ==============================
# 合并视频
# ==============================

echo
echo "开始合并视频..."

ffmpeg -y \
    -f concat \
    -safe 0 \
    -i "${LIST_FILE}" \
    -c copy \
    "${MERGED_VIDEO}"

echo
echo "视频合并完成："
echo "${MERGED_VIDEO}"


# ==============================
# 创建图片输出目录
# ==============================

mkdir -p "${IMAGE_DIR}"


# ==============================
# 抽帧
# ==============================

echo
echo "开始抽帧..."
echo "每秒 ${FPS} 帧"
echo "从 ${START_NUMBER} 开始编号"

ffmpeg -y \
    -i "${MERGED_VIDEO}" \
    -vf "fps=${FPS}" \
    -start_number "${START_NUMBER}" \
    "${IMAGE_DIR}/%0${DIGITS}d.png"


# ==============================
# 完成
# ==============================

echo
echo "======================================"
echo "处理完成"
echo "======================================"
echo "合并视频："
echo "  ${MERGED_VIDEO}"
echo
echo "图片输出目录："
echo "  ${IMAGE_DIR}"
echo
echo "图片编号示例："
printf "  %0${DIGITS}d.png\n" "${START_NUMBER}"
printf "  %0${DIGITS}d.png\n" "$((START_NUMBER + 1))"
printf "  %0${DIGITS}d.png\n" "$((START_NUMBER + 2))"
echo "======================================"