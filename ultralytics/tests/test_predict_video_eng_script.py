from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "predict_video_eng.py"
SPEC = importlib.util.spec_from_file_location("predict_video_eng_script", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_load_class_names_maps_aliases_to_english(tmp_path: Path) -> None:
    yaml_path = tmp_path / "mydata.yaml"
    yaml_path.write_text("names:\n  0: dao\n  1: fire\n", encoding="utf-8")

    names = MODULE.load_class_names(yaml_path)

    assert names == {0: "knife", 1: "fire"}


def test_build_status_lines_contains_english_runtime_state() -> None:
    lines = MODULE.build_status_lines(
        save_enabled=True,
        detection_summary="Detections: knife x1",
        queue_message="Gate queue: 2 people, ETA: 4 min",
        alert_lines=["Alert: detected 1 person carrying a knife"],
        extinguisher_message="Fire extinguisher: normal (detected: 2)",
    )

    assert "Runtime: detecting | Save: on" in lines
    assert "Alert: detected 1 person carrying a knife" in lines
    assert "Gate queue: 2 people, ETA: 4 min" in lines
    assert "Fire extinguisher: normal (detected: 2)" in lines
    assert "Detections: knife x1" in lines


def test_summarize_detections_returns_english_class_counts() -> None:
    class Boxes:
        def __init__(self) -> None:
            self.cls = self

        def tolist(self):
            return [0, 1, 1]

    class Result:
        def __init__(self) -> None:
            self.boxes = Boxes()

    summary = MODULE.summarize_detections(Result(), {0: "knife", 1: "smoke"})

    assert summary == "Detections: knife x1, smoke x2"


def test_analyze_scene_returns_english_queue_and_alerts() -> None:
    class Box:
        def __init__(self, xyxy, cls_id):
            self.xyxy = [xyxy]
            self.cls = [cls_id]

    class Result:
        def __init__(self) -> None:
            self.boxes = [
                Box([100, 100, 260, 480], 0),
                Box([120, 280, 200, 430], 1),
                Box([160, 300, 230, 450], 1),
                Box([300, 300, 360, 420], 1),
                Box([320, 120, 420, 240], 2),
                Box([30, 50, 80, 120], 3),
                Box([500, 100, 550, 220], 4),
            ]

    class_names = {0: "gate", 1: "person", 2: "smoking", 3: "knife", 4: "fire extinguisher"}
    queue_message, alert_lines, extinguisher_message = MODULE.analyze_scene(Result(), class_names)

    assert queue_message == "Gate queue: 3 people, ETA: 6 min"
    assert "Alert: detected 1 person(s) smoking" in alert_lines
    assert "Alert: detected 1 person(s) carrying a knife" in alert_lines
    assert "Alert: fire extinguisher count anomaly, 1 missing" in alert_lines
    assert extinguisher_message is None


def test_analyze_scene_keeps_legacy_chinese_labels_compatible() -> None:
    class Box:
        def __init__(self, xyxy, cls_id):
            self.xyxy = [xyxy]
            self.cls = [cls_id]

    class Result:
        def __init__(self) -> None:
            self.boxes = [
                Box([100, 100, 300, 400], 0),
                Box([140, 180, 180, 260], 1),
            ]

    class_names = {0: "电梯", 1: "灭火器"}
    queue_message, alert_lines, extinguisher_message = MODULE.analyze_scene(Result(), class_names)

    assert queue_message is None
    assert "Alert: fire extinguisher count anomaly, 1 missing" not in alert_lines
    assert extinguisher_message == "Fire extinguisher: normal (detected in elevator area)"


def test_str2bool_rejects_invalid_value() -> None:
    with pytest.raises(Exception):
        MODULE.str2bool("maybe")
