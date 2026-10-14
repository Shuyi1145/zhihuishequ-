from __future__ import annotations

import argparse
import math
import random
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
CLASS_NAMES = ["community", "non-community"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="将 raw_data 划分为训练、验证和测试集，并生成图像增强样本。")
    parser.add_argument("--new-data-root", type=Path, default=ROOT / "raw_data", help="数据目录，包含 image 和 label。")
    parser.add_argument("--output-root", type=Path, default=ROOT / "datasets" / "mydata", help="输出目录，运行时重新生成。")
    parser.add_argument("--yaml-path", type=Path, default=ROOT / "mydata.yaml", help="训练配置文件路径。")
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--val-ratio", type=float, default=0.1)
    parser.add_argument("--test-ratio", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def ensure_valid_ratios(train_ratio: float, val_ratio: float, test_ratio: float) -> None:
    ratios = (train_ratio, val_ratio, test_ratio)
    if any(not math.isfinite(value) or value < 0 for value in ratios):
        raise ValueError("划分比例必须是非负有限数值")
    if abs(sum(ratios) - 1.0) > 1e-6:
        raise ValueError("train/val/test 比例之和必须为 1.0")


def load_class_names(classes_path: Path) -> list[str]:
    names = classes_path.read_text(encoding="utf-8").splitlines()
    if len(names) != 2 or set(names) != set(CLASS_NAMES):
        raise ValueError(f"{classes_path} 必须仅包含 community 和 non-community")
    return names


def validate_label_file(label_path: Path, class_count: int) -> None:
    for line_number, line in enumerate(label_path.read_text(encoding="utf-8").splitlines(), start=1):
        parts = line.split()
        if not parts:
            continue
        try:
            if len(parts) != 5:
                raise ValueError("YOLO 检测标注需要 5 列")
            class_id = int(parts[0])
            if not 0 <= class_id < class_count:
                raise ValueError(f"类别 ID {class_id} 超出范围")
            coordinates = [float(value) for value in parts[1:]]
            if any(not math.isfinite(value) or not 0 <= value <= 1 for value in coordinates):
                raise ValueError("坐标必须在 0～1 范围内")
        except ValueError as exc:
            raise ValueError(f"{label_path} 第 {line_number} 行标注无效: {exc}") from exc


def collect_samples(data_root: Path, class_count: int) -> tuple[list[tuple[Path, Path]], list[str]]:
    image_dir, label_dir = data_root / "image", data_root / "label"
    if not image_dir.is_dir() or not label_dir.is_dir():
        raise ValueError(f"{data_root} 必须包含 image 和 label 文件夹")
    samples, missing_labels = [], []
    seen_stems = set()
    for image_path in sorted(image_dir.iterdir()):
        if not image_path.is_file() or image_path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label_path = label_dir / f"{image_path.stem}.txt"
        if not label_path.is_file():
            missing_labels.append(image_path.name)
            continue
        if image_path.stem in seen_stems:
            raise ValueError(f"存在同名图片，会共用输出标签: {image_path.stem}")
        validate_label_file(label_path, class_count)
        if cv2.imread(str(image_path)) is None:
            raise ValueError(f"无法读取图片: {image_path}")
        seen_stems.add(image_path.stem)
        samples.append((image_path, label_path))
    if not samples:
        raise ValueError(f"{data_root} 没有图片和标签配对的样本")
    return samples, missing_labels


def build_split_counts(total: int, ratios: dict[str, float]) -> dict[str, int]:
    raw_counts = {name: total * ratio for name, ratio in ratios.items()}
    counts = {name: int(value) for name, value in raw_counts.items()}
    remainder = total - sum(counts.values())
    order = sorted(ratios, key=lambda name: raw_counts[name] - counts[name], reverse=True)
    for index in range(remainder):
        counts[order[index % len(order)]] += 1
    return counts


def write_augmented_sample(
    image_path: Path,
    label_path: Path,
    image_dir: Path,
    label_dir: Path,
    seed: int = 42,
) -> None:
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"无法读取图片: {image_path}")

    original_stem = f"{image_path.stem}_orig"
    shutil.copy2(image_path, image_dir / f"{original_stem}{image_path.suffix}")
    shutil.copy2(label_path, label_dir / f"{original_stem}.txt")

    # 水平翻转时仅改变框中心的 x 坐标，类别和其他坐标保持一致。
    flipped_lines = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if parts:
            parts[1] = f"{1.0 - float(parts[1]):.6f}"
            flipped_lines.append(" ".join(parts))
    flipped_label = "\n".join(flipped_lines) + ("\n" if flipped_lines else "")
    noise_rng = np.random.default_rng(seed)
    gaussian_noise = noise_rng.normal(0, 12, image.shape).astype(np.float32)
    noisy = np.clip(image.astype(np.float32) + gaussian_noise, 0, 255).astype(np.uint8)
    variants = {
        "hflip": cv2.flip(image, 1),
        "bright": cv2.convertScaleAbs(image, alpha=1.15, beta=14),
        "dark": cv2.convertScaleAbs(image, alpha=0.65, beta=-10),
        "noise": noisy,
    }
    for name, variant in variants.items():
        stem = f"{image_path.stem}_{name}"
        image_target = image_dir / f"{stem}{image_path.suffix}"
        if not cv2.imwrite(str(image_target), variant):
            raise OSError(f"写入图片失败: {image_target}")
        label_target = label_dir / f"{stem}.txt"
        if name == "hflip":
            label_target.write_text(flipped_label, encoding="utf-8")
        else:
            shutil.copy2(label_path, label_target)


def main() -> None:
    args = parse_args()
    ensure_valid_ratios(args.train_ratio, args.val_ratio, args.test_ratio)
    data_root = args.new_data_root.resolve()
    output_root = args.output_root.resolve()
    if data_root == output_root or output_root in data_root.parents or data_root in output_root.parents:
        raise ValueError("输入目录和输出目录不能相同或互相包含")

    # 完成输入校验后才重新生成输出，避免错误标注破坏已有划分。
    class_names = load_class_names(data_root / "label" / "classes.txt")
    samples, missing_labels = collect_samples(data_root, len(class_names))
    random.Random(args.seed).shuffle(samples)
    counts = build_split_counts(len(samples), {"train": args.train_ratio, "val": args.val_ratio, "test": args.test_ratio})
    if output_root.exists():
        shutil.rmtree(output_root)

    cursor = 0
    for split, count in counts.items():
        image_dir, label_dir = output_root / split / "images", output_root / split / "labels"
        image_dir.mkdir(parents=True)
        label_dir.mkdir(parents=True)
        for sample_index, (image_path, label_path) in enumerate(samples[cursor:cursor + count], start=cursor):
            write_augmented_sample(image_path, label_path, image_dir, label_dir, seed=args.seed + sample_index)
        cursor += count

    config = {
        "path": str(output_root),
        "train": "train/images", "val": "val/images", "test": "test/images",
        "names": dict(enumerate(class_names)),
    }
    args.yaml_path.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"划分并增强完成：{len(samples)} 张原图生成 {len(samples) * 5} 个样本（原图、水平翻转、增亮、降亮度、高斯噪声）")
    print(f"train={counts['train'] * 5}，val={counts['val'] * 5}，test={counts['test'] * 5}")
    if missing_labels:
        print(f"未加入划分的无标签图片：{len(missing_labels)} 张")
    print(f"数据集：{output_root}\n训练配置：{args.yaml_path.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as exc:
        raise SystemExit(f"错误：{exc}")
