from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "predict_video.py"
SPEC = importlib.util.spec_from_file_location("predict_video_script", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_str2bool_accepts_common_values() -> None:
    assert MODULE.str2bool("true") is True
    assert MODULE.str2bool("0") is False


def test_load_class_names_supports_dict_names(tmp_path: Path) -> None:
    yaml_path = tmp_path / "mydata.yaml"
    yaml_path.write_text("names:\n  0: dao\n  1: fire\n", encoding="utf-8")

    names = MODULE.load_class_names(yaml_path)

    assert names == {0: "刀", 1: "明火"}


def test_resolve_source_keeps_camera_index() -> None:
    assert MODULE.resolve_source("0") == "0"


def test_update_result_names_updates_boxes_when_present() -> None:
    class Boxes:
        names = {}

    class Result:
        def __init__(self) -> None:
            self.names = {}
            self.boxes = Boxes()

    result = Result()
    MODULE.update_result_names(result, {0: "dao", 1: "smoke"})

    assert result.names == {0: "dao", 1: "smoke"}
    assert result.boxes.names == {0: "dao", 1: "smoke"}


def test_build_status_lines_contains_runtime_state() -> None:
    lines = MODULE.build_status_lines(
        save_enabled=True,
        detection_summary="当前识别：刀x1",
        queue_message="闸机排队人数：2人，预计等待：4分钟",
        alert_lines=["报警：检测到刀具 1 把"],
        extinguisher_message="灭火器状态：正常（检测到2个）",
    )

    assert "运行状态：检测中  保存：开启 " in lines
    assert "报警：检测到刀具 1 把" in lines
    assert "闸机排队人数：2人，预计等待：4分钟" in lines
    assert "灭火器状态：正常（检测到2个）" in lines
    assert "当前识别：刀x1" in lines
    assert all("处理进度" not in line and "实时 FPS" not in line for line in lines)


def test_summarize_detections_returns_class_counts() -> None:
    class Boxes:
        def __init__(self) -> None:
            self.cls = self

        def tolist(self):
            return [0, 1, 1]

    class Result:
        def __init__(self) -> None:
            self.boxes = Boxes()

    summary = MODULE.summarize_detections(Result(), {0: "刀", 1: "烟雾"})

    assert summary == "当前识别：刀x1、烟雾x2"


def test_analyze_scene_returns_queue_and_alerts() -> None:
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

    class_names = {0: "闸机", 1: "人", 2: "吸烟", 3: "刀", 4: "灭火器"}
    queue_message, alert_lines, extinguisher_message = MODULE.analyze_scene(Result(), class_names)

    assert queue_message == "闸机排队人数：3人，预计等待：6分钟"
    assert "报警：检测到有1人吸烟行为" in alert_lines
    assert "报警：检测到有1人手持刀具" in alert_lines
    assert "报警：灭火器数量异常，缺少一个灭火器" in alert_lines
    assert extinguisher_message is None


def test_analyze_scene_requires_gate_for_queue_estimate() -> None:
    class Box:
        def __init__(self, xyxy, cls_id):
            self.xyxy = [xyxy]
            self.cls = [cls_id]

    class Result:
        def __init__(self) -> None:
            self.boxes = [
                Box([120, 280, 200, 430], 1),
                Box([160, 300, 230, 450], 1),
            ]

    class_names = {1: "人"}
    queue_message, alert_lines, extinguisher_message = MODULE.analyze_scene(Result(), class_names)

    assert queue_message is None
    assert alert_lines == []
    assert extinguisher_message is None


def test_analyze_scene_caps_gate_queue_count_at_four() -> None:
    class Box:
        def __init__(self, xyxy, cls_id):
            self.xyxy = [xyxy]
            self.cls = [cls_id]

    class Result:
        def __init__(self) -> None:
            self.boxes = [
                Box([100, 100, 400, 500], 0),
                Box([110, 120, 150, 220], 1),
                Box([160, 140, 200, 240], 1),
                Box([210, 160, 250, 260], 1),
                Box([260, 180, 300, 280], 1),
                Box([310, 200, 350, 300], 1),
            ]

    class_names = {0: "闸机", 1: "人"}
    queue_message, _, extinguisher_message = MODULE.analyze_scene(Result(), class_names)

    assert queue_message == "闸机排队人数：4人，预计等待：8分钟"
    assert extinguisher_message is None


def test_analyze_scene_does_not_treat_smoke_as_smoking_alert() -> None:
    class Box:
        def __init__(self, xyxy, cls_id):
            self.xyxy = [xyxy]
            self.cls = [cls_id]

    class Result:
        def __init__(self) -> None:
            self.boxes = [Box([100, 100, 200, 200], 0)]

    class_names = {0: "烟雾"}
    queue_message, alert_lines, extinguisher_message = MODULE.analyze_scene(Result(), class_names)

    assert queue_message is None
    assert alert_lines == []
    assert extinguisher_message is None


def test_analyze_scene_marks_extinguisher_area_normal_with_two() -> None:
    class Box:
        def __init__(self, xyxy, cls_id):
            self.xyxy = [xyxy]
            self.cls = [cls_id]

    class Result:
        def __init__(self) -> None:
            self.boxes = [
                Box([100, 100, 160, 220], 0),
                Box([200, 100, 260, 220], 0),
            ]

    class_names = {0: "灭火器"}
    queue_message, alert_lines, extinguisher_message = MODULE.analyze_scene(Result(), class_names)

    assert queue_message is None
    assert alert_lines == []
    assert extinguisher_message == "灭火器状态：正常（检测到2个）"


def test_analyze_scene_marks_extinguisher_normal_when_inside_elevator() -> None:
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
    assert "报警：灭火器数量异常，缺少一个灭火器" not in alert_lines
    assert extinguisher_message == "灭火器状态：正常（电梯区域内检测到灭火器）"


def test_str2bool_rejects_invalid_value() -> None:
    with pytest.raises(Exception):
        MODULE.str2bool("maybe")
