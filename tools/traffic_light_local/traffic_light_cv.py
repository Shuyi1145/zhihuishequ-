"""本地三灯识别演示。只处理图片，不向机器人发送控制命令。"""

import argparse
from dataclasses import dataclass
from itertools import combinations
import json
from pathlib import Path
import time

import cv2
import numpy as np


LABELS = {"red": "红", "yellow": "黄", "green": "绿", "unknown": "未知"}


@dataclass
class Settings:
    # 以下是当前截图的起始参数，其他相机/光照需要重新标定。
    roi_bottom: float = 0.45
    radius_min: float = 0.038
    radius_max: float = 0.095
    circle_threshold: float = 27
    bright_value: int = 210
    bright_fraction: float = 0.15
    color_fraction: float = 0.25
    brightness_margin: float = 25


def read_image(path):
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("图片无法解码，可能正在写入或格式不正确")
    return image


def lamp_stats(hsv, circle, settings):
    x, y, radius = circle
    height, width = hsv.shape[:2]
    x0, x1 = max(0, int(x-radius)), min(width, int(x+radius)+1)
    y0, y1 = max(0, int(y-radius)), min(height, int(y+radius)+1)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    mask = (xx-x)**2 + (yy-y)**2 <= (radius * 0.8)**2
    pixels = hsv[y0:y1, x0:x1][mask]
    hue, saturation, value = pixels.T
    colored = (saturation >= 70) & (value >= 40)
    fractions = {
        "red": float(np.mean(colored & ((hue <= 12) | (hue >= 170)))),
        "yellow": float(np.mean(colored & (hue >= 13) & (hue <= 38))),
        "green": float(np.mean(colored & (hue >= 39) & (hue <= 95))),
    }
    color = max(fractions, key=fractions.get)
    return {
        "circle": [float(v) for v in circle],
        "color": color if fractions[color] >= settings.color_fraction else "unknown",
        "color_fraction": fractions[color],
        "bright_fraction": float(np.mean(value >= settings.bright_value)),
        "v80": float(np.percentile(value, 80)),
    }


def group_error(group):
    """筛选近似等半径、共线、等间距的三灯组合，允许横排和竖排。"""
    circles = np.array([lamp["circle"] for lamp in group])
    centers, radii = circles[:, :2], circles[:, 2]
    if radii.max() / radii.min() > 1.45:
        return None
    distances = np.linalg.norm(centers[:, None] - centers[None, :], axis=2)
    a, b = np.unravel_index(np.argmax(distances), distances.shape)
    middle = 3 - a - b
    direction = centers[b] - centers[a]
    offset = centers[middle] - centers[a]
    line_error = abs(direction[0]*offset[1]-direction[1]*offset[0]) / distances[a, b]
    d1, d2 = distances[a, middle], distances[middle, b]
    radius = radii.mean()
    if line_error > radius * 0.4 or max(d1, d2) / min(d1, d2) > 1.4:
        return None
    if not (2.2 * radius <= min(d1, d2) and max(d1, d2) <= 6 * radius):
        return None
    if {lamp["color"] for lamp in group} != {"red", "yellow", "green"}:
        return None
    return float(line_error/radius + abs(d1-d2)/max(d1, d2))


def detect(image, settings=None):
    settings = settings or Settings()
    height, width = image.shape[:2]
    scale = min(height, width)
    bottom = int(height * settings.roi_bottom)
    roi = image[:bottom]
    gray = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5, 5), 1.2)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=scale*0.065,
        param1=100, param2=settings.circle_threshold,
        minRadius=max(3, round(scale*settings.radius_min)),
        maxRadius=max(4, round(scale*settings.radius_max)),
    )
    result = {"state": "unknown", "reason": "没有找到可靠的三灯组合", "lamps": []}
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (width-1, bottom-1), (255, 180, 0), 1)
    if circles is None:
        return result, overlay
    # 候选过多时拒绝判断，避免组合数过大或在杂乱背景中误选。
    if len(circles[0]) > 40:
        result["reason"] = "圆候选过多，请收紧搜索区域或半径范围"
        return result, overlay
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lamps = [lamp_stats(hsv, circle, settings) for circle in circles[0]]
    groups = [group for group in combinations(lamps, 3) if group_error(group) is not None]
    if len(groups) != 1:
        if len(groups) > 1:
            result["reason"] = "存在多个灯组候选，不能确定相关灯组"
        return result, overlay
    selected = list(groups[0])
    result["lamps"] = selected
    lit = [lamp for lamp in selected if lamp["bright_fraction"] >= settings.bright_fraction]
    if len(lit) == 1:
        active = lit[0]
        dark_v = max(lamp["v80"] for lamp in selected if lamp is not active)
        if active["v80"] - dark_v >= settings.brightness_margin:
            result.update(state=active["color"], reason="三灯结构、颜色、亮灭及亮度差校验通过")
        else:
            result["reason"] = "亮暗差异不足"
    else:
        result["reason"] = "没有唯一亮灯"
    for lamp in selected:
        x, y, radius = [round(v) for v in lamp["circle"]]
        cv2.circle(overlay, (x, y), radius, (0, 255, 255), 2)
        cv2.circle(overlay, (x, y), round(radius*0.8), (255, 0, 255), 1)
        text = f'{lamp["color"]}: bright={lamp["bright_fraction"]:.2f}'
        cv2.putText(overlay, text, (max(0, x-radius), y+radius+18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 0), 1)
    cv2.putText(overlay, result["state"].upper(), (10, max(20, bottom+25)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return result, overlay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, default=Path(__file__).with_name("红绿灯测试.png"))
    parser.add_argument("--interval", type=float, default=1, help="等待状态下的重读间隔，秒")
    parser.add_argument("--max-checks", type=int, default=0, help="0 表示等待直到绿灯；正数用于有限次验证")
    parser.add_argument("--roi-bottom", type=float, default=0.45, help="搜索范围底边占全图高度的比例")
    parser.add_argument("--show", action="store_true", help="显示检测画面，按 Q 退出")
    args = parser.parse_args()
    if args.interval <= 0 or args.max_checks < 0 or not 0 < args.roi_bottom <= 1:
        parser.error("interval 必须大于 0，max-checks 不小于 0，roi-bottom 在 (0, 1] 内")
    settings = Settings(roi_bottom=args.roi_bottom)
    output = Path(__file__).with_name("检测输出")
    output.mkdir(exist_ok=True)
    checks = 0
    try:
        while True:
            try:
                frame = read_image(args.image)
                result, overlay = detect(frame, settings)
            except (OSError, ValueError, cv2.error) as exc:
                result = {"state": "unknown", "reason": str(exc), "lamps": []}
                overlay = None
            state = result["state"]
            allowed = state == "green"
            print(f"当前是：{LABELS[state]}灯", flush=True)
            print("通行：" + ("可通行" if allowed else "不可通行"), flush=True)
            print("通过中.....。结束检测" if allowed else "等待中.....。", flush=True)
            (output / "检测结果.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
            if overlay is not None:
                cv2.imencode(".png", overlay)[1].tofile(str(output / "检测标注.png"))
                if args.show:
                    cv2.imshow("Traffic light CV", overlay)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            checks += 1
            if allowed:
                break
            if args.max_checks and checks >= args.max_checks:
                print("达到测试次数上限，停止本地测试；未允许通行。", flush=True)
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n已手动停止检测。", flush=True)
    finally:
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
