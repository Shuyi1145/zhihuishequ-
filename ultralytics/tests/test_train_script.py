from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "train.py"
SPEC = importlib.util.spec_from_file_location("custom_train_script", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_str2bool_accepts_common_values() -> None:
    assert MODULE.str2bool("true") is True
    assert MODULE.str2bool("1") is True
    assert MODULE.str2bool("false") is False
    assert MODULE.str2bool("0") is False


def test_collect_environment_info_has_expected_keys() -> None:
    env = MODULE.collect_environment_info(None)

    assert "python" in env
    assert "torch" in env
    assert "cuda_available" in env
    assert "resolved_device" in env
    assert isinstance(env["will_use_gpu"], bool)


def test_default_output_belongs_to_current_project(monkeypatch, tmp_path) -> None:
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sys, "argv", [str(MODULE_PATH)])
    args = MODULE.parse_args()
    kwargs = MODULE.build_train_kwargs(args, "cpu")

    assert kwargs["project"] == str(MODULE_PATH.parent / "runs" / "detect")
    assert kwargs["data"] == str(MODULE_PATH.parent / "mydata.yaml")
    assert kwargs["exist_ok"] is False


def test_relative_project_is_resolved_against_script_directory(monkeypatch) -> None:
    monkeypatch.setattr(sys, "argv", [str(MODULE_PATH), "--project", "custom_runs"])
    kwargs = MODULE.build_train_kwargs(MODULE.parse_args(), "cpu")

    assert kwargs["project"] == str(MODULE_PATH.parent / "custom_runs")


def test_existing_run_creates_separate_directory(monkeypatch, tmp_path) -> None:
    from types import SimpleNamespace
    from ultralytics.cfg import get_save_dir

    (tmp_path / "train").mkdir()
    marker = tmp_path / "train" / "old_result.jpg"
    marker.write_bytes(b"previous run")
    monkeypatch.setattr(sys, "argv", [str(MODULE_PATH), "--project", str(tmp_path)])
    kwargs = MODULE.build_train_kwargs(MODULE.parse_args(), "cpu")
    args = SimpleNamespace(project=kwargs["project"], name=kwargs["name"], exist_ok=kwargs["exist_ok"], task="detect", mode="train")

    assert get_save_dir(args) == tmp_path / "train2"
    assert marker.read_bytes() == b"previous run"


def test_light_training_uses_current_project_and_separate_run(monkeypatch) -> None:
    import runpy
    from unittest.mock import Mock
    import ultralytics

    model = Mock()
    yolo = Mock(return_value=model)
    monkeypatch.setattr(ultralytics, "YOLO", yolo)

    runpy.run_path(str(MODULE_PATH.parent / "train_ligth.py"))

    yolo.assert_called_once_with(str(MODULE_PATH.parent / "yolo26s.pt"))
    kwargs = model.train.call_args.kwargs
    assert kwargs["data"] == str(MODULE_PATH.parent / "mydata.yaml")
    assert kwargs["project"] == str(MODULE_PATH.parent / "runs" / "detect")
    assert kwargs["exist_ok"] is False
