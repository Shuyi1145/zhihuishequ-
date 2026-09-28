#!/usr/bin/env python3
"""Create a two-class YOLO dataset from Gazebo backgrounds and RGBA people."""

import argparse
import random
from pathlib import Path

import cv2
import numpy as np


CLASS_NAMES = ("community", "non-community")


def read_image(path, flags):
    """Read paths containing non-ASCII characters on every OpenCV platform."""
    encoded = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(encoded, flags) if encoded.size else None


def write_jpeg(path, image):
    success, encoded = cv2.imencode(".jpg", image,
                                    [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not success:
        return False
    encoded.tofile(str(path))
    return True


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--models-dir", required=True, type=Path)
    parser.add_argument("--backgrounds-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--train-count", type=int, default=400)
    parser.add_argument("--val-count", type=int, default=100)
    parser.add_argument("--min-people", type=int, default=2)
    parser.add_argument("--max-people", type=int, default=7)
    parser.add_argument("--min-height-ratio", type=float, default=0.28)
    parser.add_argument("--max-height-ratio", type=float, default=0.72)
    parser.add_argument("--seed", type=int, default=20260928)
    return parser.parse_args()


def validate_args(args):
    if args.train_count <= 0 or args.val_count <= 0:
        raise ValueError("train-count and val-count must be positive")
    if not 1 <= args.min_people <= args.max_people:
        raise ValueError("people count range is invalid")
    if not 0 < args.min_height_ratio <= args.max_height_ratio <= 1:
        raise ValueError("height ratios must satisfy 0 < min <= max <= 1")


def discover_assets(models_dir):
    patterns = (
        "person_community_*/materials/textures/*.png",
        "person_noncommunity_*/materials/textures/*.png",
    )
    assets = []
    for class_id, pattern in enumerate(patterns):
        for path in sorted(models_dir.glob(pattern)):
            image = read_image(path, cv2.IMREAD_UNCHANGED)
            if image is None or image.ndim != 3 or image.shape[2] != 4:
                raise ValueError("RGBA person image required: {}".format(path))
            alpha = image[:, :, 3]
            ys, xs = np.where(alpha > 10)
            if not len(xs):
                raise ValueError("person image has no visible pixels: {}".format(path))
            image = image[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
            assets.append((class_id, path, image))
    grouped = {class_id: [] for class_id in range(len(CLASS_NAMES))}
    for class_id, path, image in assets:
        grouped[class_id].append((path, image))
    if any(not grouped[class_id] for class_id in grouped):
        raise ValueError("both community and non-community assets are required")
    return grouped


def discover_backgrounds(backgrounds_dir):
    extensions = {".jpg", ".jpeg", ".png", ".bmp"}
    paths = sorted(path for path in backgrounds_dir.rglob("*")
                   if path.is_file() and path.suffix.lower() in extensions)
    valid = []
    for path in paths:
        image = read_image(path, cv2.IMREAD_COLOR)
        if image is not None and image.shape[0] >= 128 and image.shape[1] >= 128:
            valid.append(path)
    if not valid:
        raise ValueError("no readable background images found")
    return valid


def rotate_bound(image, angle):
    height, width = image.shape[:2]
    center = (width / 2.0, height / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    cosine = abs(matrix[0, 0])
    sine = abs(matrix[0, 1])
    new_width = int(height * sine + width * cosine)
    new_height = int(height * cosine + width * sine)
    matrix[0, 2] += new_width / 2.0 - center[0]
    matrix[1, 2] += new_height / 2.0 - center[1]
    return cv2.warpAffine(image, matrix, (new_width, new_height),
                          flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT,
                          borderValue=(0, 0, 0, 0))


def augment_person(source, target_height, rng):
    scale = target_height / source.shape[0]
    target_width = max(1, int(round(source.shape[1] * scale)))
    person = cv2.resize(source, (target_width, target_height),
                        interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
    person = rotate_bound(person, rng.uniform(-4.0, 4.0))
    gain = rng.uniform(0.78, 1.18)
    offset = rng.uniform(-12.0, 12.0)
    colors = person[:, :, :3].astype(np.float32) * gain + offset
    person[:, :, :3] = np.clip(colors, 0, 255).astype(np.uint8)
    if rng.random() < 0.20:
        person[:, :, :3] = cv2.GaussianBlur(person[:, :, :3], (3, 3), 0)
    return person


def intersection_over_union(first, second):
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    first_area = (first[2] - first[0]) * (first[3] - first[1])
    second_area = (second[2] - second[0]) * (second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union else 0.0


def composite(background, person, left, top):
    height, width = person.shape[:2]
    alpha = person[:, :, 3:4].astype(np.float32) / 255.0
    region = background[top:top + height, left:left + width].astype(np.float32)
    foreground = person[:, :, :3].astype(np.float32)
    background[top:top + height, left:left + width] = np.clip(
        foreground * alpha + region * (1.0 - alpha), 0, 255).astype(np.uint8)


def normalized_label(class_id, box, image_width, image_height):
    left, top, right, bottom = box
    center_x = (left + right) / 2.0 / image_width
    center_y = (top + bottom) / 2.0 / image_height
    width = (right - left) / image_width
    height = (bottom - top) / image_height
    return "{} {:.6f} {:.6f} {:.6f} {:.6f}".format(
        class_id, center_x, center_y, width, height)


def render_scene(background_path, assets, args, rng):
    image = read_image(background_path, cv2.IMREAD_COLOR)
    height, width = image.shape[:2]
    image = np.clip(image.astype(np.float32) * rng.uniform(0.88, 1.12),
                    0, 255).astype(np.uint8)
    count = rng.randint(args.min_people, args.max_people)
    class_ids = [0, 1] if count >= 2 else [rng.randrange(2)]
    class_ids.extend(rng.randrange(2) for _ in range(count - len(class_ids)))
    rng.shuffle(class_ids)
    placed_boxes = []
    labels = []

    for class_id in class_ids:
        _, source = rng.choice(assets[class_id])
        target_height = int(height * rng.uniform(args.min_height_ratio,
                                                 args.max_height_ratio))
        person = augment_person(source.copy(), max(16, target_height), rng)
        person_height, person_width = person.shape[:2]
        if person_height >= height or person_width >= width:
            continue
        mask = person[:, :, 3] > 15
        ys, xs = np.where(mask)
        if not len(xs):
            continue
        for _ in range(40):
            left = rng.randint(0, width - person_width)
            bottom = rng.randint(max(person_height, int(height * 0.72)), height)
            top = bottom - person_height
            box = (left + int(xs.min()), top + int(ys.min()),
                   left + int(xs.max()) + 1, top + int(ys.max()) + 1)
            if all(intersection_over_union(box, existing) <= 0.28
                   for existing in placed_boxes):
                break
        else:
            continue
        composite(image, person, left, top)
        placed_boxes.append(box)
        labels.append(normalized_label(class_id, box, width, height))

    if not labels:
        raise RuntimeError("failed to place any person on {}".format(background_path))
    return image, labels


def write_split(split, count, backgrounds, assets, args, rng):
    image_dir = args.output_dir / "images" / split
    label_dir = args.output_dir / "labels" / split
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    totals = [0, 0]
    for index in range(count):
        image, labels = render_scene(rng.choice(backgrounds), assets, args, rng)
        stem = "synthetic_{:06d}".format(index)
        if not write_jpeg(image_dir / (stem + ".jpg"), image):
            raise IOError("failed to write generated image")
        (label_dir / (stem + ".txt")).write_text("\n".join(labels) + "\n",
                                                  encoding="utf-8")
        for label in labels:
            totals[int(label.split()[0])] += 1
    return totals


def main():
    args = parse_args()
    validate_args(args)
    args.models_dir = args.models_dir.expanduser().resolve()
    args.backgrounds_dir = args.backgrounds_dir.expanduser().resolve()
    args.output_dir = args.output_dir.expanduser().resolve()
    rng = random.Random(args.seed)
    assets = discover_assets(args.models_dir)
    backgrounds = discover_backgrounds(args.backgrounds_dir)
    rng.shuffle(backgrounds)
    if len(backgrounds) >= 2:
        split_at = max(1, int(round(len(backgrounds) * 0.8)))
        split_at = min(split_at, len(backgrounds) - 1)
        train_backgrounds = backgrounds[:split_at]
        val_backgrounds = backgrounds[split_at:]
    else:
        train_backgrounds = val_backgrounds = backgrounds
        print("WARNING: only one background; validation will not be independent")

    train_totals = write_split("train", args.train_count, train_backgrounds,
                               assets, args, rng)
    val_totals = write_split("val", args.val_count, val_backgrounds,
                             assets, args, rng)
    yaml_text = (
        "path: {}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: community\n"
        "  1: non-community\n"
    ).format(args.output_dir.as_posix())
    (args.output_dir / "persons.yaml").write_text(yaml_text, encoding="utf-8")
    print("assets: community={}, non-community={}".format(
        len(assets[0]), len(assets[1])))
    print("backgrounds: train={}, val={}".format(
        len(train_backgrounds), len(val_backgrounds)))
    print("train instances: community={}, non-community={}".format(*train_totals))
    print("val instances: community={}, non-community={}".format(*val_totals))
    print("dataset: {}".format(args.output_dir))


if __name__ == "__main__":
    main()
