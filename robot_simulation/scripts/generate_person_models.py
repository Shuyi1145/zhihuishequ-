#!/usr/bin/env python3
"""Generate Gazebo Classic placards from the competition person images."""

import argparse
import re
import tempfile
from pathlib import Path

from PIL import Image

from generate_gazebo_placard import create_model, validate_dimensions
from configure_person_back import configure_person_back


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PACKAGE_DIR.parents[1] / "复赛资料" / "人员"
DEFAULT_MODELS = PACKAGE_DIR / "models"
SOURCE_FOLDERS = (("社区人员", "community"), ("非社区人员", "noncommunity"))


def model_name(category, image_path, suffix=""):
    stem = image_path.stem.lower()
    if not re.fullmatch(r"[a-z0-9_]+", stem):
        raise ValueError("图片文件名只能包含英文字母、数字和下划线：{}".format(image_path))
    if suffix and not re.fullmatch(r"_[a-z0-9_]+", suffix):
        raise ValueError("模型名称后缀只能包含小写英文字母、数字和下划线")
    if stem.isdigit():
        stem = "{:02d}".format(int(stem))
    return "person_{}_{}{}".format(category, stem, suffix)


def collect_images(source_root, suffix="", only=None):
    models = []
    names = set()
    remaining = set(only) if only is not None else None
    for folder, category in SOURCE_FOLDERS:
        directory = source_root / folder
        if not directory.is_dir():
            raise ValueError("找不到人员素材目录：{}".format(directory))
        images = sorted(directory.glob("*.png"))
        if not images:
            raise ValueError("目录中没有 PNG 图片：{}".format(directory))
        for image_path in images:
            if remaining is not None and image_path.stem.lower() not in remaining:
                continue
            name = model_name(category, image_path, suffix=suffix)
            if name in names:
                raise ValueError("模型名称重复：{}".format(name))
            names.add(name)
            with Image.open(image_path) as image:
                if "A" not in image.getbands():
                    raise ValueError("图片缺少透明通道：{}".format(image_path))
                image.verify()
            models.append((image_path, name))
            if remaining is not None:
                remaining.remove(image_path.stem.lower())
    if remaining:
        raise ValueError("找不到指定人员图片：{}".format(", ".join(sorted(remaining))))
    if not models:
        raise ValueError("没有符合筛选条件的人员图片")
    return models


def _white_background(image_path, destination):
    with Image.open(image_path) as image:
        source = image.convert("RGBA")
        background = Image.new("RGBA", source.size, (255, 255, 255, 255))
        Image.alpha_composite(background, source).save(destination, format="PNG")


def generate_models(source_root, models_dir, height=0.15, thickness=0.005,
                    force=False, width=0.05, white_background=False,
                    model_suffix=None, only=None):
    validate_dimensions(width, thickness, height)
    if model_suffix is None:
        suffix = "_white" if white_background else ""
    else:
        suffix = model_suffix
    if white_background and not suffix:
        raise ValueError("白底模型必须使用独立模型名称后缀")
    models = collect_images(Path(source_root), suffix=suffix, only=only)
    models_dir = Path(models_dir)
    counts = {"created": 0, "updated": 0, "skipped": 0}
    with tempfile.TemporaryDirectory(prefix=".person-textures-") as temporary:
        processed = Path(temporary)
        for image_path, name in models:
            source_image = image_path
            if white_background:
                source_image = processed / "{}.png".format(name)
                _white_background(image_path, source_image)
            result = create_model(source_image, models_dir, name, width,
                                  thickness, height, overwrite=force,
                                  texture_name=image_path.name)
            if result != "skipped":
                configure_person_back(models_dir / name)
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
    parser.add_argument("--white-background", action="store_true",
                        help="将透明区域合成为白色，并给模型名称添加 _white 后缀")
    parser.add_argument("--model-suffix", default=None,
                        help="覆盖模型名称后缀；白底模型必须保留独立后缀")
    parser.add_argument("--only", action="append", default=None,
                        help="只生成指定图片名（不含扩展名），可重复传入")
    args = parser.parse_args()
    try:
        counts = generate_models(args.source_root, args.models_dir,
                                 height=args.height, thickness=args.thickness,
                                 force=args.force, width=args.width,
                                 white_background=args.white_background,
                                 model_suffix=args.model_suffix,
                                 only=args.only)
    except (OSError, ValueError) as exc:
        parser.exit(1, "生成失败：{}\n".format(exc))
    print("完成：新增 {created}，更新 {updated}，跳过 {skipped}".format(**counts))


if __name__ == "__main__":
    main()
