import importlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
generator = importlib.import_module("generate_person_models")
generate_models = generator.generate_models
model_name = generator.model_name


class GeneratePersonModelsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "people"
        self.models = self.root / "models"
        (self.source / "社区人员").mkdir(parents=True)
        (self.source / "非社区人员").mkdir(parents=True)
        Image.new("RGBA", (155, 352), (255, 0, 0, 0)).save(
            self.source / "社区人员" / "1.png")
        Image.new("RGBA", (121, 365), (0, 255, 0, 0)).save(
            self.source / "非社区人员" / "F1.png")

    def test_generates_visual_collision_and_transparency(self):
        self.assertEqual(generate_models(self.source, self.models),
                         {"created": 2, "updated": 0, "skipped": 0})
        for name, image_name in (("person_community_01", "1.png"),
                                 ("person_noncommunity_f1", "F1.png")):
            model = self.models / name
            sdf = ET.parse(model / "model.sdf").getroot()
            self.assertEqual(sdf.find(".//collision/geometry/box/size").text,
                             "0.05 0.005 0.15")
            self.assertEqual(sdf.find(".//collision/pose").text,
                             "0 0 0.075 0 0 0")
            self.assertEqual(sdf.find(".//visual/geometry/mesh/uri").text,
                             "model://{}/meshes/{}.dae".format(name, name))
            dae = ET.parse(model / "meshes" / "{}.dae".format(name)).getroot()
            ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
            positions = [float(value) for value in dae.find(
                ".//c:float_array[@id='positions_array']", ns).text.split()]
            self.assertEqual(min(positions[0::3]), -0.025)
            self.assertEqual(max(positions[0::3]), 0.025)
            triangles = dae.find(".//c:triangles", ns)
            indices = triangles.find("c:p", ns).text.split()
            self.assertEqual(len(indices), int(triangles.attrib["count"]) * 9)
            self.assertEqual(dae.find(".//c:image/c:init_from", ns).text,
                             image_name)
            self.assertEqual((model / "meshes" / image_name).read_bytes(),
                             (model / "materials" / "textures" /
                              image_name).read_bytes())
            material = (model / "materials" / "scripts" /
                        "{}.material".format(name)).read_text()
            self.assertIn("alpha_rejection greater_equal 128", material)
            self.assertIn("texture {}".format(image_name), material)
            self.assertIn("cull_hardware clockwise", material)
            self.assertNotIn("cull_hardware none", material)
            self.assertEqual(triangles.attrib["count"], "2")
            back = sdf.find(".//visual[@name='white_back']")
            self.assertEqual(back.find("material/diffuse").text, "1 1 1 1")
            self.assertEqual(back.find("material/lighting").text, "false")
            self.assertIsNone(back.find("material/script"))
            back_mesh = ET.parse(model / "meshes" / (name + "_back.dae")).getroot()
            self.assertIsNone(back_mesh.find(".//c:texture", ns))
            self.assertEqual(back_mesh.find(".//c:triangles", ns).attrib["count"], "2")
            back_positions = [float(value) for value in back_mesh.find(
                ".//c:float_array[@id='positions_array']", ns).text.split()]
            self.assertEqual(back_positions[1::3], [0.0025] * 4)
            # 正反三角形绕序及法向分别对应 -Y、+Y。
            for mesh, expected_y in ((dae, -1), (back_mesh, 1)):
                vertices = [float(value) for value in mesh.find(
                    ".//c:float_array[@id='positions_array']", ns).text.split()]
                normals = [float(value) for value in mesh.find(
                    ".//c:float_array[@id='normals_array']", ns).text.split()]
                faces = [int(value) for value in mesh.find(".//c:triangles/c:p", ns).text.split()]
                for offset in (0, 9):
                    a, b, c = [vertices[faces[offset + step] * 3:faces[offset + step] * 3 + 3]
                               for step in (0, 3, 6)]
                    cross_y = (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2])
                    self.assertGreater(cross_y * expected_y, 0)
                    self.assertEqual(normals[faces[offset + 1] * 3 + 1], expected_y)

    def test_white_back_world_update_is_repeatable_and_preserves_poses(self):
        generate_models(self.source, self.models)
        backs = importlib.import_module("configure_person_back")
        model = self.models / "person_community_01"
        front_before = (model / "meshes/1.png").read_bytes()
        world_path = self.root / "test.world"
        world_path.write_text(
            '<sdf version="1.5"><world name="test"><model name="person_copy">'
            '<pose>1 2 0 0 0 1.57</pose><link name="link"><visual name="visual">'
            '<geometry><mesh><uri>model://person_community_01/meshes/person_community_01.dae</uri>'
            '</mesh></geometry></visual></link></model></world></sdf>')
        for _ in range(2):
            back = backs.configure_person_back(model)
            backs.update_world(world_path, {model.name: back})
        world = ET.parse(world_path).getroot()
        self.assertEqual(len(world.findall(".//visual[@name='white_back']")), 1)
        self.assertEqual(world.find(".//model/pose").text, "1 2 0 0 0 1.57")
        self.assertEqual((model / "meshes/1.png").read_bytes(), front_before)

    def test_existing_models_are_preserved_unless_forced(self):
        generate_models(self.source, self.models)
        sdf_path = self.models / "person_community_01" / "model.sdf"
        sdf_path.write_text("custom", encoding="utf-8")
        self.assertEqual(generate_models(self.source, self.models),
                         {"created": 0, "updated": 0, "skipped": 2})
        self.assertEqual(sdf_path.read_text(encoding="utf-8"), "custom")
        self.assertEqual(generate_models(self.source, self.models, force=True),
                         {"created": 0, "updated": 2, "skipped": 0})
        self.assertEqual(ET.parse(sdf_path).getroot().tag, "sdf")

    def test_white_background_creates_separate_opaque_models(self):
        self.assertEqual(
            generate_models(self.source, self.models,
                            white_background=True),
            {"created": 2, "updated": 0, "skipped": 0})
        for name, image_name in (("person_community_01_white", "1.png"),
                                 ("person_noncommunity_f1_white", "F1.png")):
            texture = self.models / name / "materials" / "textures" / image_name
            image = Image.open(texture).convert("RGBA")
            self.assertTrue(all(alpha == 255 for alpha in image.getchannel("A").getdata()))
            self.assertEqual(image.getpixel((0, 0)), (255, 255, 255, 255))
            self.assertEqual(
                ET.parse(self.models / name / "model.sdf").getroot().find(
                    ".//visual/geometry/mesh/uri").text,
                "model://{}/meshes/{}.dae".format(name, name))

    def test_white_background_can_be_limited_to_requested_people(self):
        Image.new("RGBA", (155, 352), (255, 0, 0, 0)).save(
            self.source / "社区人员" / "9.png")
        Image.new("RGBA", (155, 352), (0, 0, 255, 0)).save(
            self.source / "社区人员" / "16.png")
        self.assertEqual(
            generate_models(self.source, self.models,
                            white_background=True, only=["9", "16"]),
            {"created": 2, "updated": 0, "skipped": 0})
        self.assertEqual(sorted(path.name for path in self.models.iterdir()),
                         ["person_community_09_white",
                          "person_community_16_white"])

    def test_white_background_filter_rejects_unknown_people(self):
        with self.assertRaisesRegex(ValueError, "找不到指定人员图片"):
            generate_models(self.source, self.models,
                            white_background=True, only=["99"])

    def test_white_background_cannot_override_original_model_names(self):
        with self.assertRaisesRegex(ValueError, "独立模型名称后缀"):
            generate_models(self.source, self.models,
                            white_background=True, model_suffix="")
        self.assertFalse(self.models.exists())

    def test_rejects_images_without_alpha_before_writing_models(self):
        Image.new("RGB", (10, 20), "white").save(
            self.source / "社区人员" / "2.png")
        with self.assertRaisesRegex(ValueError, "透明通道"):
            generate_models(self.source, self.models)
        self.assertFalse(self.models.exists())

    def test_rejects_invalid_size_and_name(self):
        with self.assertRaisesRegex(ValueError, "宽度"):
            generate_models(self.source, self.models, width=0)
        with self.assertRaisesRegex(ValueError, "高度"):
            generate_models(self.source, self.models, height=0)
        with self.assertRaisesRegex(ValueError, "厚度"):
            generate_models(self.source, self.models, thickness=float("nan"))
        with self.assertRaisesRegex(ValueError, "文件名"):
            model_name("community", Path("姓名.png"))


if __name__ == "__main__":
    unittest.main()
