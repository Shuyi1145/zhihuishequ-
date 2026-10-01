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
