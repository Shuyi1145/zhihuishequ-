#!/usr/bin/env python3
"""Build a Gazebo Classic visual and collision model from one image."""

import argparse
import hashlib
import math
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image, ImageOps


COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
TEMPLATE = Path(__file__).resolve().parent / "templates" / "placard.dae"
ET.register_namespace("", COLLADA_NS)


def default_model_name(image_path):
    stem = re.sub(r"[^a-z0-9]+", "_", Path(image_path).stem.lower()).strip("_")
    if not stem:
        stem = hashlib.sha1(Path(image_path).stem.encode("utf-8")).hexdigest()[:8]
    return "placard_{}".format(stem)


def validate_dimensions(width, thickness, height):
    for label, value in (("宽度", width), ("厚度", thickness),
                         ("高度", height)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("{} 必须是正数，单位为米".format(label))


def _add(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def _write_sdf(path, name, material_name, width, thickness, height):
    sdf = ET.Element("sdf", version="1.5")
    model = _add(sdf, "model", name=name)
    _add(model, "static", "true")
    link = _add(model, "link", name="link")
    visual = _add(link, "visual", name="visual")
    mesh = _add(_add(visual, "geometry"), "mesh")
    _add(mesh, "uri", "model://{}/meshes/{}.dae".format(name, name))
    script = _add(_add(visual, "material"), "script")
    _add(script, "uri", "model://{}/materials/scripts".format(name))
    _add(script, "uri", "model://{}/materials/textures".format(name))
    _add(script, "name", material_name)
    collision = _add(link, "collision", name="collision")
    box = _add(_add(collision, "geometry"), "box")
    _add(box, "size", "{} {} {}".format(width, thickness, height))
    _add(collision, "pose", "0 0 {} 0 0 0".format(height / 2))
    ET.ElementTree(sdf).write(path, encoding="utf-8", xml_declaration=True)


def _write_config(path, name):
    model = ET.Element("model")
    _add(model, "name", name)
    _add(model, "version", "1.0")
    _add(model, "sdf", "model.sdf", version="1.5")
    _add(model, "description", "Static image placard with box collision.")
    ET.ElementTree(model).write(path, encoding="utf-8", xml_declaration=True)


def _write_dae(path, texture_name, width, height):
    tree = ET.parse(TEMPLATE)
    root = tree.getroot()
    namespace = {"c": COLLADA_NS}
    root.find(".//c:image/c:init_from", namespace).text = texture_name
    positions = root.find(".//c:float_array[@id='positions_array']", namespace)
    half_width = width / 2
    positions.text = "{} 0 0 {} 0 0 {} 0 {} {} 0 {}".format(
        -half_width, half_width, half_width, height, -half_width, height
    )
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _write_material(path, material_name, texture_name):
    path.write_text(
        "material {}\n"
        "{{\n  technique\n  {{\n    pass\n    {{\n"
        "      lighting on\n      scene_blend alpha_blend\n"
        "      alpha_rejection greater_equal 128\n"
        "      depth_write off\n      cull_hardware none\n"
        "      cull_software none\n\n      texture_unit\n"
        "      {{\n        texture {}\n      }}\n"
        "    }}\n  }}\n}}\n".format(material_name, texture_name),
        encoding="utf-8"
    )


def _write_texture(image_path, mesh_path, material_path):
    with Image.open(image_path) as image:
        if image.format == "PNG" and image.mode == "RGBA" and not image.getexif().get(274):
            shutil.copy2(image_path, mesh_path)
        else:
            ImageOps.exif_transpose(image).convert("RGBA").save(mesh_path)
    shutil.copy2(mesh_path, material_path)


def create_model(image_path, models_dir, name, width, thickness, height,
                 overwrite=False, texture_name=None):
    """Create one model and return created, updated, or skipped."""
    image_path = Path(image_path)
    models_dir = Path(models_dir)
    validate_dimensions(width, thickness, height)
    if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        raise ValueError("模型名只能使用小写英文字母、数字和下划线，并以字母开头")
    if not image_path.is_file():
        raise ValueError("找不到图片：{}".format(image_path))
    texture_name = texture_name or "{}.png".format(name)
    if not re.fullmatch(r"[A-Za-z0-9_-]+\.png", texture_name):
        raise ValueError("纹理文件名必须是简单的 PNG 文件名")
    with Image.open(image_path) as image:
        image.verify()

    destination = models_dir / name
    if destination.exists():
        if not destination.is_dir():
            raise ValueError("模型路径不是目录：{}".format(destination))
        if not overwrite:
            return "skipped"
    models_dir.mkdir(parents=True, exist_ok=True)
    material_name = "".join(part.capitalize() for part in name.split("_"))
    with tempfile.TemporaryDirectory(prefix=".{}-".format(name),
                                     dir=models_dir) as temporary:
        staged = Path(temporary)
        mesh_dir = staged / "meshes"
        script_dir = staged / "materials" / "scripts"
        texture_dir = staged / "materials" / "textures"
        for directory in (mesh_dir, script_dir, texture_dir):
            directory.mkdir(parents=True)
        _write_sdf(staged / "model.sdf", name, material_name,
                   width, thickness, height)
        _write_config(staged / "model.config", name)
        _write_dae(mesh_dir / "{}.dae".format(name), texture_name, width, height)
        _write_material(script_dir / "{}.material".format(name),
                        material_name, texture_name)
        _write_texture(image_path, mesh_dir / texture_name,
                       texture_dir / texture_name)
        if destination.exists():
            for generated in staged.rglob("*"):
                if generated.is_file():
                    target = destination / generated.relative_to(staged)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    os.replace(generated, target)
            return "updated"
        staged.rename(destination)
    return "created"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True, help="PNG、JPEG 等图片路径")
    parser.add_argument("--width", type=float, required=True, help="X 轴宽度，单位 m")
    parser.add_argument("--thickness", type=float, required=True,
                        help="Y 轴厚度，单位 m")
    parser.add_argument("--height", type=float, required=True, help="Z 轴高度，单位 m")
    parser.add_argument("--name", help="Gazebo 模型名，默认根据图片文件名生成")
    parser.add_argument("--output-dir", type=Path, default=Path.cwd() / "models",
                        help="Gazebo 模型父目录，默认 ./models")
    parser.add_argument("--overwrite", action="store_true", help="更新同名模型")
    args = parser.parse_args()
    name = args.name or default_model_name(args.image)
    try:
        result = create_model(args.image, args.output_dir, name,
                              args.width, args.thickness, args.height,
                              overwrite=args.overwrite)
    except (OSError, ValueError) as exc:
        parser.exit(1, "生成失败：{}\n".format(exc))
    print("{}：{}".format(result, args.output_dir / name))


if __name__ == "__main__":
    main()
