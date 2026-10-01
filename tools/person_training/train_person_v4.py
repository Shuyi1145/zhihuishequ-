"""Train the next person detector using the reviewed local dataset.

Run with the y8 Conda environment on Windows. Use --check-only to validate
the dataset without starting training.
"""

import argparse
import hashlib
import math
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "dataset"
DATA_CONFIG = DATASET / "persons.yaml"
DEFAULT_MODEL = ROOT / "runs" / "person_v3" / "weights" / "best.pt"
RUNS_DIR = ROOT / "runs"
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
CLASS_NAMES = {0: "community", 1: "non-community"}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true", help="Validate data, then exit")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--name", default="person_v4", help="New run directory name")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--patience", type=int, default=10)
    parser.add_argument("--batch", type=int, default=8)
    return parser.parse_args()


def index_images(directory):
    images = {}
    for path in directory.iterdir():
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS:
            if path.stem in images:
                raise ValueError(f"同一目录有重名图片：{images[path.stem]} 和 {path}")
            images[path.stem] = path
    return images


def validate_labels(labels):
    counts = {class_id: 0 for class_id in CLASS_NAMES}
    for path in labels.values():
        for line_number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
            if not line.strip():
                continue
            parts = line.split()
            try:
                if len(parts) != 5:
                    raise ValueError("应有 5 列")
                class_id = int(parts[0])
                x, y, width, height = map(float, parts[1:])
                if class_id not in CLASS_NAMES:
                    raise ValueError("类别必须是 0 或 1")
                if not all(math.isfinite(value) for value in (x, y, width, height)):
                    raise ValueError("坐标包含非有限数值")
                if width <= 0 or height <= 0:
                    raise ValueError("框宽高必须大于 0")
                tolerance = 1e-6  # Six-decimal YOLO labels can round at image edges.
                if (x - width / 2 < -tolerance or x + width / 2 > 1 + tolerance
                        or y - height / 2 < -tolerance or y + height / 2 > 1 + tolerance):
                    raise ValueError("框超出图片边界")
            except ValueError as error:
                raise ValueError(f"标签错误：{path}:{line_number}: {error}") from error
            counts[class_id] += 1
    return counts


def validate_dataset():
    if not DATA_CONFIG.is_file():
        raise FileNotFoundError(f"缺少数据配置：{DATA_CONFIG}")
    config = yaml.safe_load(DATA_CONFIG.read_text(encoding="utf-8"))
    names = {int(key): value for key, value in config.get("names", {}).items()}
    if names != CLASS_NAMES:
        raise ValueError(f"类别配置不符：{names}")
    if Path(config.get("path", "")).resolve() != DATASET:
        raise ValueError("persons.yaml 中的 path 与当前数据集目录不一致")
    if config.get("train") != "images/train" or config.get("val") != "images/val":
        raise ValueError("persons.yaml 中的 train/val 路径与本脚本检查的目录不一致")

    hashes = {}
    for split in ("train", "val"):
        image_dir = DATASET / "images" / split
        label_dir = DATASET / "labels" / split
        if not image_dir.is_dir() or not label_dir.is_dir():
            raise FileNotFoundError(f"缺少 {split} 图片或标签目录")
        images = index_images(image_dir)
        labels = {path.stem: path for path in label_dir.glob("*.txt")}
        if not images:
            raise ValueError(f"{split} 没有图片")
        if images.keys() != labels.keys():
            missing = sorted(images.keys() - labels.keys())
            orphaned = sorted(labels.keys() - images.keys())
            raise ValueError(f"{split} 图片/标签不配对：缺标签 {missing}；多余标签 {orphaned}")
        counts = validate_labels(labels)
        print(f"{split}: {len(images)} 张图片，{len(labels)} 个标签文件，框数 {counts}")

        for image in images.values():
            digest = hashlib.sha256(image.read_bytes()).hexdigest()
            previous = hashes.get(digest)
            if previous and previous[0] != split:
                raise ValueError(f"训练/验证集存在相同图片：{previous[1]} 和 {image}")
            hashes[digest] = (split, image)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    args = parse_args()
    validate_dataset()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"找不到起始权重：{model_path}")
    if args.check_only:
        print("检查通过，未启动训练。")
        return

    if not args.name or args.name in {".", ".."} or Path(args.name).name != args.name:
        raise ValueError("--name 必须是单个目录名")
    if min(args.epochs, args.patience, args.batch) <= 0:
        raise ValueError("epochs、patience、batch 必须为正整数")
    run_dir = RUNS_DIR / args.name
    if run_dir.exists():
        raise FileExistsError(f"结果目录已存在，避免覆盖：{run_dir}；请换一个 --name")

    import torch
    from ultralytics import YOLO

    if not torch.cuda.is_available():
        raise RuntimeError("未检测到 CUDA GPU；本脚本不会自动退回到耗时的 CPU 训练")

    print(f"起始权重：{model_path}")
    print(f"训练结果将保存到：{run_dir}")
    model = YOLO(str(model_path))
    if model.names != CLASS_NAMES:
        raise ValueError(f"权重类别与数据集不一致：{model.names}")
    model.train(
        data=str(DATA_CONFIG),
        epochs=args.epochs,
        patience=args.patience,
        imgsz=640,
        batch=args.batch,
        device=0,
        workers=0,
        optimizer="AdamW",
        lr0=0.0003,
        freeze=0,
        mosaic=0.0,
        seed=45,
        plots=True,
        project=str(RUNS_DIR),
        name=args.name,
        exist_ok=False,
    )
    print(f"训练完成。最佳权重：{run_dir / 'weights' / 'best.pt'}")


if __name__ == "__main__":
    main()
