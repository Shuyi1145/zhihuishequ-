import importlib.util
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image, ImageChops, ImageStat


ROOT = Path(__file__).resolve().parents[1]
CURRENT = ROOT / "models/traffic_light"
ANIMATED = ROOT / "models/traffic_light_animated"
spec = importlib.util.spec_from_file_location(
    "generate_traffic_light_animation",
    ROOT / "scripts/generate_traffic_light_animation.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class AnimatedTrafficLightModelTest(unittest.TestCase):
    def test_geometry_matches_current_model_with_visual_player(self):
        current = ET.parse(CURRENT / "model.sdf").getroot()
        animated = ET.parse(ANIMATED / "model.sdf").getroot()
        self.assertIsNone(animated.find("./model/plugin"))
        plugin = animated.find(".//visual[@name='animated_face']/plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.get("filename"), "libtraffic_light_animation.so")
        for tag in ("visual", "collision"):
            originals = {
                item.get("name"): item
                for item in current.findall(".//link/" + tag)
                if not item.get("name").endswith("_lamp")
            }
            copies = {
                item.get("name"): item
                for item in animated.findall(".//link/" + tag)
                if item.get("name") != "animated_face"
            }
            self.assertEqual(originals.keys(), copies.keys())
            for name in originals:
                self.assertEqual(originals[name].findtext("pose"),
                                 copies[name].findtext("pose"))
                self.assertEqual(
                    ET.tostring(originals[name].find("geometry")),
                    ET.tostring(copies[name].find("geometry")))
        self.assertEqual(animated.findtext(".//link/gravity"), "false")
        self.assertEqual(animated.findtext(".//link/kinematic"), "true")

    def test_single_material_and_frames_follow_red_yellow_green_cycle(self):
        material = (ANIMATED / "materials/scripts/traffic_light_animated.material").read_text()
        animated = ET.parse(ANIMATED / "model.sdf").getroot()
        self.assertEqual(material.count("anim_texture"), 1)
        sequence = next(line.split()[1:] for line in material.splitlines()
                        if line.strip().startswith("anim_texture "))
        # 帧时钟由 VisualPlugin 接管，必须禁止原生动画同时写帧号。
        self.assertEqual(float(sequence[-1]), 0.0)
        self.assertEqual(sequence[:-1], [
            "traffic_light_animated_{}_v2.png".format(color)
            for color in ("red", "red", "yellow", "green", "green", "green")])
        self.assertEqual(generator.FRAME_STATES,
                         ("red", "red", "yellow", "green", "green", "green"))
        face = animated.find(".//visual[@name='animated_face']")
        self.assertEqual(face.findtext("geometry/plane/size"), "0.59 0.14")
        self.assertEqual(face.findtext("material/script/name"),
                         "TrafficLightAnimated/FaceRYGSimTime")

        for active in generator.COLORS:
            path = ANIMATED / "materials/textures/traffic_light_animated_{}_v2.png".format(active)
            with Image.open(path) as frame:
                self.assertEqual(frame.size, generator.FACE_SIZE)
                expected = generator.build_frame(active)
                self.assertIsNone(ImageChops.difference(frame, expected).getbbox())
                brightness = {}
                for color, center in zip(generator.COLORS, generator.CENTERS):
                    patch = frame.crop((center - 40, 100, center + 40, 180))
                    brightness[color] = sum(ImageStat.Stat(patch).mean)
                for color in generator.COLORS:
                    if color != active:
                        self.assertGreater(brightness[active], brightness[color] + 70)


if __name__ == "__main__":
    unittest.main()
