#!/usr/bin/env python3
"""Keep person placard textures on the front and make the back white."""

import argparse
import copy
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from generate_gazebo_placard import COLLADA_NS


NS = {"c": COLLADA_NS}
PACKAGE_DIR = Path(__file__).resolve().parents[1]


def configure_person_back(model_dir):
    """更新现有立牌，保留正面纹理、尺寸及碰撞体。"""
    model_dir = Path(model_dir)
    sdf_path = model_dir / "model.sdf"
    sdf_tree = ET.parse(sdf_path)
    link = sdf_tree.getroot().find("model/link")
    visual = link.find("visual")
    mesh_uri = visual.find("geometry/mesh/uri").text
    mesh_path = model_dir / "meshes" / Path(mesh_uri).name
    mesh_tree = ET.parse(mesh_path)
    triangles = mesh_tree.getroot().find(".//c:triangles", NS)
    # 原网格前两组三角形朝向 -Y，后两组朝向 +Y。
    indices = triangles.find("c:p", NS).text.split()
    triangles.set("count", "2")
    triangles.find("c:p", NS).text = " ".join(indices[:18])

    back_tree = copy.deepcopy(mesh_tree)
    back_root = back_tree.getroot()
    back_triangles = back_root.find(".//c:triangles", NS)
    back_triangles.find("c:p", NS).text = (
        "3 1 3 2 1 2 1 1 1 3 1 3 1 1 1 0 1 0")
    # 背面与正面错开，避免共面闪烁，尺寸仍取当前模型数据。
    thickness = float(link.find("collision/geometry/box/size").text.split()[1])
    positions = back_root.find(".//c:float_array[@id='positions_array']", NS)
    coordinates = positions.text.split()
    for index in range(1, len(coordinates), 3):
        coordinates[index] = str(thickness / 2)
    positions.text = " ".join(coordinates)
    profile = back_root.find(".//c:profile_COMMON", NS)
    for param in list(profile.findall("c:newparam", NS)):
        profile.remove(param)
    lambert = profile.find("c:technique/c:lambert", NS)
    for child in list(lambert):
        lambert.remove(child)
    diffuse = ET.SubElement(lambert, "{%s}diffuse" % COLLADA_NS)
    ET.SubElement(diffuse, "{%s}color" % COLLADA_NS).text = "1 1 1 1"
    images = back_root.find("c:library_images", NS)
    if images is not None:
        back_root.remove(images)

    material_path = model_dir / "materials/scripts" / (model_dir.name + ".material")
    material = material_path.read_text(encoding="utf-8")
    # 去掉旧的双面渲染注释，启用背面剔除。
    material = re.sub(r"//[^\n]*", "", material)
    material = material.replace("cull_hardware none", "cull_hardware clockwise")
    material = material.replace("cull_software none", "cull_software back")

    back = link.find("visual[@name='white_back']")
    if back is not None:
        link.remove(back)
    back = ET.SubElement(link, "visual", name="white_back")
    geometry = ET.SubElement(back, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    back_name = mesh_path.stem + "_back.dae"
    ET.SubElement(mesh, "uri").text = mesh_uri.rsplit("/", 1)[0] + "/" + back_name
    if visual.find("pose") is not None:
        back.append(copy.deepcopy(visual.find("pose")))
    if visual.find("geometry/mesh/scale") is not None:
        mesh.append(copy.deepcopy(visual.find("geometry/mesh/scale")))
    white = ET.SubElement(back, "material")
    ET.SubElement(white, "lighting").text = "false"
    for tag in ("ambient", "diffuse", "emissive"):
        ET.SubElement(white, tag).text = "1 1 1 1"
    ET.SubElement(white, "specular").text = "0 0 0 1"

    mesh_tree.write(mesh_path, encoding="utf-8", xml_declaration=True)
    back_tree.write(mesh_path.with_name(back_name), encoding="utf-8", xml_declaration=True)
    material_path.write_text(material, encoding="utf-8")
    sdf_tree.write(sdf_path, encoding="utf-8", xml_declaration=True)
    return back


def update_world(world_path, backs):
    """同步 world 中已展开的行人模型，仅插入背面 visual。"""
    world_path = Path(world_path)
    content = world_path.read_text(encoding="utf-8")
    pattern = r"(<visual\b[^>]*>.*?<uri>model://([^/]+)/meshes/[^<]+</uri>.*?</visual>)"

    def add_back(match):
        back = backs.get(match.group(2))
        if back is None:
            return match.group(0)
        return match.group(0) + "\n        " + ET.tostring(back, encoding="unicode")

    # 重复运行不产生重复的背面。
    content = re.sub(r"\s*<visual name=['\"]white_back['\"]>.*?</visual>",
                     "", content, flags=re.DOTALL)
    content = re.sub(pattern, add_back, content, flags=re.DOTALL)
    ET.fromstring(content)
    world_path.write_text(content, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models-dir", type=Path, default=PACKAGE_DIR / "models")
    parser.add_argument("--world", type=Path,
                        default=PACKAGE_DIR / "world/competition.world")
    args = parser.parse_args()
    backs = {}
    for sdf_path in sorted(args.models_dir.rglob("person_*/model.sdf")):
        backs[sdf_path.parent.name] = configure_person_back(sdf_path.parent)
    update_world(args.world, backs)
    print("Updated %d person models and %s" % (len(backs), args.world))


if __name__ == "__main__":
    main()
