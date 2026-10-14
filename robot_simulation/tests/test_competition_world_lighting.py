import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORLD = Path(__file__).resolve().parents[1] / "world" / "competition.world"


class CompetitionWorldLightingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        world = ET.parse(WORLD).getroot().find("world")
        cls.lights = {
            light.get("name"): light
            for light in world.findall("light")
        }

    def test_side_light_matches_original_configuration(self):
        self.assertTrue(
            {"side_light"} <= self.lights.keys()
        )
        self.assertNotIn("person_fill_light", self.lights)
        light = self.lights["side_light"]
        self.assertEqual(light.get("type"), "point")
        self.assertEqual(light.findtext("pose"), "3 2 1 0 -0 0")
        self.assertEqual(light.findtext("diffuse"), "1 1 1 1")
        self.assertEqual(light.findtext("attenuation/linear"), "0")
        self.assertEqual(light.findtext("attenuation/quadratic"), "0")
        self.assertEqual(light.findtext("attenuation/range"), "10")
        self.assertEqual(light.findtext("range"), "5.0")

if __name__ == "__main__":
    unittest.main()
