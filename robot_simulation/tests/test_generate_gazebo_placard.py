import importlib
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
generator = importlib.import_module("generate_gazebo_placard")


class GenerateGazeboPlacardTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.models = self.root / "models"

    def test_custom_dimensions_control_visual_and_collision(self):
        image_path = self.root / "人物.png"
        Image.new("RGBA", (80, 160), (255, 0, 0, 0)).save(image_path)
        self.assertEqual(generator.create_model(
            image_path, self.models, "test_person", 0.08, 0.007, 0.21),
            "created")
        model = self.models / "test_person"
        sdf = ET.parse(model / "model.sdf").getroot()
        self.assertEqual(sdf.find(".//collision/geometry/box/size").text,
                         "0.08 0.007 0.21")
        self.assertEqual(sdf.find(".//collision/pose").text,
                         "0 0 0.105 0 0 0")
        ns = {"c": generator.COLLADA_NS}
        dae = ET.parse(model / "meshes/test_person.dae").getroot()
        positions = [float(value) for value in dae.find(
            ".//c:float_array[@id='positions_array']", ns).text.split()]
        self.assertEqual(min(positions[0::3]), -0.04)
        self.assertEqual(max(positions[0::3]), 0.04)
        self.assertEqual(max(positions[2::3]), 0.21)
        triangles = dae.find(".//c:triangles", ns)
        self.assertEqual(len(triangles.find("c:p", ns).text.split()), 36)
        self.assertEqual(dae.find(".//c:image/c:init_from", ns).text,
                         "test_person.png")
        material = (model / "materials/scripts/test_person.material").read_text()
        self.assertIn("alpha_rejection greater_equal 128", material)
        with Image.open(model / "materials/textures/test_person.png") as texture:
            self.assertEqual(texture.mode, "RGBA")
            self.assertEqual(texture.getpixel((0, 0))[3], 0)

    def test_rgb_jpeg_is_converted_and_unicode_name_gets_default(self):
        image_path = self.root / "红灯.jpg"
        Image.new("RGB", (12, 24), "red").save(image_path)
        name = generator.default_model_name(image_path)
        self.assertTrue(name.startswith("placard_"))
        self.assertEqual(generator.create_model(
            image_path, self.models, name, 0.1, 0.01, 0.2), "created")
        with Image.open(self.models / name / "meshes" / (name + ".png")) as image:
            self.assertEqual(image.mode, "RGBA")
            self.assertEqual(image.size, (12, 24))
            self.assertEqual(image.getpixel((0, 0))[3], 255)

    def test_existing_model_requires_overwrite(self):
        image_path = self.root / "image.png"
        Image.new("RGBA", (10, 20), "white").save(image_path)
        generator.create_model(image_path, self.models, "sign", 0.05, 0.005, 0.15)
        sdf_path = self.models / "sign/model.sdf"
        self.assertEqual(generator.create_model(
            image_path, self.models, "sign", 0.08, 0.005, 0.15), "skipped")
        self.assertIn("0.05 0.005 0.15", sdf_path.read_text())
        self.assertEqual(generator.create_model(
            image_path, self.models, "sign", 0.08, 0.005, 0.15,
            overwrite=True), "updated")
        self.assertIn("0.08 0.005 0.15", sdf_path.read_text())

    def test_rejects_invalid_dimensions_name_and_image(self):
        image_path = self.root / "image.png"
        Image.new("RGBA", (10, 20), "white").save(image_path)
        with self.assertRaisesRegex(ValueError, "厚度"):
            generator.create_model(image_path, self.models, "sign", 0.05, 0, 0.15)
        with self.assertRaisesRegex(ValueError, "模型名"):
            generator.create_model(image_path, self.models, "../sign", 0.05, 0.005, 0.15)
        with self.assertRaisesRegex(ValueError, "找不到图片"):
            generator.create_model(self.root / "missing.png", self.models,
                                   "sign", 0.05, 0.005, 0.15)
        self.assertFalse(self.models.exists())


if __name__ == "__main__":
    unittest.main()
