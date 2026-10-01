#!/usr/bin/env python3
"""Generate Gazebo Classic placards from the competition person images."""

import argparse
import re
from pathlib import Path

from PIL import Image

from generate_gazebo_placard import create_model, validate_dimensions


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PACKAGE_DIR.parents[1] / "复赛资料" / "人员"
DEFAULT_MODELS = PACKAGE_DIR / "models"
SOURCE_FOLDERS = (("社区人员", "community"), ("非社区人员", "noncommunity"))


def model_name(category, image_path):
    stem = image_path.stem.lower()
    if not re.fullmatch(r"[a-z0-9_]+", stem):
        raise ValueError("图片文件名只能包含英文字母、数字和下划线：{}".format(image_path))
    if stem.isdigit():
        stem = "{:02d}".format(int(stem))
    return "person_{}_{}".format(category, stem)


def collect_images(source_root):
    models = []
    names = set()
    for folder, category in SOURCE_FOLDERS:
        directory = source_root / folder
        if not directory.is_dir():
            raise ValueError("找不到人员素材目录：{}".format(directory))
        images = sorted(directory.glob("*.png"))
        if not images:
            raise ValueError("目录中没有 PNG 图片：{}".format(directory))
        for image_path in images:
            name = model_name(category, image_path)
            if name in names:
                raise ValueError("模型名称重复：{}".format(name))
            names.add(name)
            with Image.open(image_path) as image:
                if "A" not in image.getbands():
                    raise ValueError("图片缺少透明通道：{}".format(image_path))
                image.verify()
            models.append((image_path, name))
    return models


def generate_models(source_root, models_dir, height=0.15, thickness=0.005,
                    force=False, width=0.05):
    validate_dimensions(width, thickness, height)
    models = collect_images(Path(source_root))
    models_dir = Path(models_dir)
    counts = {"created": 0, "updated": 0, "skipped": 0}
    for image_path, name in models:
        result = create_model(image_path, models_dir, name, width, thickness,
                              height, overwrite=force,
                              texture_name=image_path.name)
        counts[result] += 1
        print("{}：{}（{} × {} × {} m）".format(
            result, name, width, thickness, height))
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE,
                        help="包含社区人员和非社区人员子目录的素材目录")
    parser.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS,
                        help="Gazebo 模型输出目录")
    parser.add_argument("--height", type=float, default=0.15,
                        help="立牌高度，单位 m，默认 0.15")
    parser.add_argument("--width", type=float, default=0.05,
                        help="立牌宽度，单位 m，默认 0.05")
    parser.add_argument("--thickness", type=float, default=0.005,
                        help="碰撞体厚度，单位 m，默认 0.005")
    parser.add_argument("--force", action="store_true",
                        help="更新已存在模型中的生成文件")
    args = parser.parse_args()
    try:
        counts = generate_models(args.source_root, args.models_dir,
                                 height=args.height, thickness=args.thickness,
                                 force=args.force, width=args.width)
    except (OSError, ValueError) as exc:
        parser.exit(1, "生成失败：{}\n".format(exc))
    print("完成：新增 {created}，更新 {updated}，跳过 {skipped}".format(**counts))


if __name__ == "__main__":
    main()
