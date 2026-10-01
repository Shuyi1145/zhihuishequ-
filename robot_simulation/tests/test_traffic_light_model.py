import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image


MODEL = Path(__file__).resolve().parents[1] / "models" / "traffic_light"
WORLD = Path(__file__).resolve().parents[1] / "world" / "competition.world"


class TrafficLightModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = ET.parse(MODEL / "model.sdf").getroot()
        cls.visuals = {
            visual.get("name"): visual
            for visual in cls.sdf.findall(".//link/visual")
        }
        cls.collisions = {
            collision.get("name"): collision
            for collision in cls.sdf.findall(".//link/collision")
        }

    def test_housing_and_legs_have_matching_collision_and_dimensions(self):
        expected = {
            "housing": ((0, 0, 0.41), (0.59, 0.05, 0.14)),
            "left_leg": ((-0.3075, 0, 0.24), (0.025, 0.025, 0.48)),
            "right_leg": ((0.3075, 0, 0.24), (0.025, 0.025, 0.48)),
        }
        bounds = []
        for name, (position, size) in expected.items():
            for element in (self.visuals[name], self.collisions[name + "_collision"]):
                self.assertEqual(
                    tuple(map(float, element.find("geometry/box/size").text.split())),
                    size)
                self.assertEqual(
                    tuple(map(float, element.find("pose").text.split()[:3])),
                    position)
            bounds.append(tuple((p - s / 2, p + s / 2)
                                for p, s in zip(position, size)))

        for axis, extent in enumerate((0.64, 0.05, 0.48)):
            low = min(bound[axis][0] for bound in bounds)
            high = max(bound[axis][1] for bound in bounds)
            self.assertAlmostEqual(high - low, extent)
            if axis == 2:
                self.assertAlmostEqual(low, 0)

    def test_three_lamps_start_with_only_red_on_and_have_both_textures(self):
        materials = (MODEL / "materials/scripts/traffic_light.material").read_text()
        self.assertEqual({name for name in self.visuals if name.endswith("_lamp")},
                         {"red_lamp", "yellow_lamp", "green_lamp"})
        for color, x in (("red", -0.2), ("yellow", 0), ("green", 0.2)):
            visual = self.visuals[color + "_lamp"]
            pose = tuple(map(float, visual.find("pose").text.split()))
            self.assertEqual(pose[:3], (x, -0.026, 0.41))
            self.assertAlmostEqual(pose[3], math.pi / 2)
            self.assertEqual(pose[4:], (0, 0))
            self.assertEqual(visual.find("geometry/plane/normal").text,
                             "0 0 1")
            self.assertEqual(visual.find("geometry/plane/size").text,
                             "0.13 0.13")
            self.assertEqual(visual.find("material/script/name").text,
                             "TrafficLight/" + color.title() +
                             ("On" if color == "red" else "Dark"))
            for state in ("dark", "on"):
                texture = color + "_" + state + ".png"
                self.assertIn("texture " + texture, materials)
                with Image.open(MODEL / "materials/textures" / texture) as image:
                    self.assertEqual(image.size, (1024, 1024))

    def test_plugin_cycle_is_ten_fifteen_five_seconds(self):
        plugin = self.sdf.find(".//plugin[@name='traffic_light_controller']")
        self.assertEqual(plugin.get("filename"), "libtraffic_light_controller.so")
        self.assertEqual([float(plugin.find(name + "_duration").text)
                          for name in ("red", "green", "yellow")],
                         [10, 15, 5])

    def test_model_and_world_allow_pose_manipulation_without_gravity(self):
        world = ET.parse(WORLD).getroot()
        for model in (self.sdf.find("model"),
                      world.find("./world/model[@name='traffic_light']")):
            self.assertIsNotNone(model)
            self.assertIn(model.findtext("static"), ("false", "0"))
            body = model.find("link[@name='body']")
            self.assertIn(body.findtext("gravity"), ("false", "0"))
            self.assertIn(body.findtext("kinematic"), ("true", "1"))


if __name__ == "__main__":
    unittest.main()
