import importlib.util
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "prepare_mydata_dataset.py"
SPEC = importlib.util.spec_from_file_location("prepare_mydata_dataset", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_default_paths_and_ratios(monkeypatch):
    monkeypatch.setattr(sys, "argv", [str(MODULE_PATH)])
    args = MODULE.parse_args()
    assert args.new_data_root == MODULE.ROOT / "raw_data"
    assert args.output_root == MODULE.ROOT / "datasets" / "mydata"
    assert (args.train_ratio, args.val_ratio, args.test_ratio) == (0.8, 0.1, 0.1)


@pytest.mark.parametrize("ratios", [(0.8, 0.1, 0.2), (-0.1, 0.1, 1), (float("nan"), 0, 1), (float("inf"), 0, 0)])
def test_rejects_invalid_ratios(ratios):
    with pytest.raises(ValueError):
        MODULE.ensure_valid_ratios(*ratios)


@pytest.mark.parametrize("total", [0, 1, 7, 102])
def test_split_counts_keep_all_samples(total):
    counts = MODULE.build_split_counts(total, {"train": 0.8, "val": 0.1, "test": 0.1})
    assert sum(counts.values()) == total
    assert all(count >= 0 for count in counts.values())
    if total == 102:
        assert counts == {"train": 82, "val": 10, "test": 10}


@pytest.mark.parametrize("names", ["", "community\nother\n", "community\ncommunity\n"])
def test_rejects_wrong_classes(tmp_path, names):
    path = tmp_path / "classes.txt"
    path.write_text(names)
    with pytest.raises(ValueError):
        MODULE.load_class_names(path)


def test_reads_class_order(tmp_path):
    path = tmp_path / "classes.txt"
    path.write_text("non-community\ncommunity\n")
    assert MODULE.load_class_names(path) == ["non-community", "community"]


@pytest.mark.parametrize("line", [
    "2 0.1 0.2 0.3 0.4", "-1 0.1 0.2 0.3 0.4", "x 0.1 0.2 0.3 0.4",
    "0 0.1", "0 nan 0.2 0.3 0.4", "0 1.2 0.2 0.3 0.4", "0 x 0.2 0.3 0.4",
])
def test_invalid_label_reports_path_and_line(tmp_path, line):
    path = tmp_path / "invalid.txt"
    path.write_text(line + "\n")
    with pytest.raises(ValueError, match="invalid.txt 第 1 行"):
        MODULE.validate_label_file(path, 2)


@pytest.fixture
def raw_data(tmp_path):
    root = tmp_path / "raw_data"
    (root / "image").mkdir(parents=True)
    (root / "label").mkdir()
    (root / "label" / "classes.txt").write_text("community\nnon-community\n")
    for index in range(10):
        image = np.full((8, 10, 3), index * 10, dtype=np.uint8)
        MODULE.cv2.imwrite(str(root / "image" / f"{index}.jpg"), image)
        (root / "label" / f"{index}.txt").write_text(f"{index % 2} 0.1 0.2 0.3 0.4\n\n")
    return root


def test_collect_samples_skips_missing_labels_and_accepts_empty_labels(raw_data):
    (raw_data / "image" / "missing.png").write_bytes(b"no label")
    (raw_data / "image" / "notes.md").write_text("ignore")
    (raw_data / "label" / "0.txt").write_text("")
    samples, missing = MODULE.collect_samples(raw_data, 2)
    assert len(samples) == 10
    assert missing == ["missing.png"]


def test_collect_samples_rejects_duplicate_stems(raw_data):
    (raw_data / "image" / "0.png").write_bytes(b"duplicate stem")
    with pytest.raises(ValueError, match="同名图片"):
        MODULE.collect_samples(raw_data, 2)


def test_collect_samples_rejects_missing_or_empty_directories(tmp_path):
    with pytest.raises(ValueError, match="文件夹"):
        MODULE.collect_samples(tmp_path, 2)
    (tmp_path / "image").mkdir()
    (tmp_path / "label").mkdir()
    with pytest.raises(ValueError, match="没有"):
        MODULE.collect_samples(tmp_path, 2)


def test_cli_augments_without_split_leakage_and_reruns_without_accumulating(raw_data, tmp_path):
    output = tmp_path / "dataset"
    yaml_path = tmp_path / "mydata.yaml"
    command = [sys.executable, str(MODULE_PATH), "--new-data-root", str(raw_data),
               "--output-root", str(output), "--yaml-path", str(yaml_path)]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    before = {path.relative_to(output): path.read_bytes() for path in output.rglob("*") if path.is_file()}
    stems = []
    for split, count in {"train": 8, "val": 1, "test": 1}.items():
        images = list((output / split / "images").glob("*.jpg"))
        assert len(images) == count * 5
        assert len(list((output / split / "labels").glob("*.txt"))) == count * 5
        for original in (output / split / "images").glob("*_orig.jpg"):
            stem = original.stem.removesuffix("_orig")
            stems.append(stem)
            assert original.read_bytes() == (raw_data / "image" / f"{stem}.jpg").read_bytes()
            label_text = (raw_data / "label" / f"{stem}.txt").read_text()
            for variant in ("orig", "hflip", "bright", "dark", "noise"):
                assert (output / split / "images" / f"{stem}_{variant}.jpg").exists()
                label = output / split / "labels" / f"{stem}_{variant}.txt"
                if variant == "hflip":
                    assert float(label.read_text().split()[1]) == pytest.approx(0.9)
                    assert label.read_text().split()[0] == label_text.split()[0]
                else:
                    assert label.read_text() == label_text
    assert len(stems) == len(set(stems)) == 10
    config = MODULE.yaml.safe_load(yaml_path.read_text())
    assert config["names"] == {0: "community", 1: "non-community"}
    assert config["path"] == str(output)
    (output / "obsolete.txt").write_text("old output")
    subprocess.run(command, check=True, capture_output=True)
    after = {path.relative_to(output): path.read_bytes() for path in output.rglob("*") if path.is_file()}
    assert before == after


def test_augmentation_changes_pixels_and_preserves_box_geometry(tmp_path):
    image_path, label_path = tmp_path / "sample.png", tmp_path / "sample.txt"
    image = np.arange(60, dtype=np.uint8).reshape(4, 5, 3)
    MODULE.cv2.imwrite(str(image_path), image)
    label_path.write_text("0 0.2 0.3 0.4 0.5\n1 0.7 0.6 0.2 0.1\n")
    images, labels = tmp_path / "images", tmp_path / "labels"
    images.mkdir()
    labels.mkdir()
    MODULE.write_augmented_sample(image_path, label_path, images, labels)
    np.testing.assert_array_equal(MODULE.cv2.imread(str(images / "sample_hflip.png")), image[:, ::-1])
    np.testing.assert_array_equal(
        MODULE.cv2.imread(str(images / "sample_bright.png")), MODULE.cv2.convertScaleAbs(image, alpha=1.15, beta=14)
    )
    dark = MODULE.cv2.imread(str(images / "sample_dark.png"))
    assert dark.mean() < image.mean()
    noise = MODULE.cv2.imread(str(images / "sample_noise.png"))
    assert not np.array_equal(noise, image)
    assert (images / "sample_orig.png").read_bytes() == image_path.read_bytes()
    for variant in ("bright", "dark", "noise"):
        assert (labels / f"sample_{variant}.txt").read_text() == label_path.read_text()
    assert (labels / "sample_hflip.txt").read_text() == "0 0.800000 0.3 0.4 0.5\n1 0.300000 0.6 0.2 0.1\n"


def test_augmentation_handles_empty_labels(raw_data, tmp_path):
    label_path = raw_data / "label" / "0.txt"
    label_path.write_text("")
    images, labels = tmp_path / "images", tmp_path / "labels"
    images.mkdir()
    labels.mkdir()
    MODULE.write_augmented_sample(raw_data / "image" / "0.jpg", label_path, images, labels)
    assert len(list(labels.iterdir())) == 5
    assert all(path.read_bytes() == b"" for path in labels.iterdir())


def test_rejects_unreadable_image(raw_data):
    (raw_data / "image" / "0.jpg").write_bytes(b"invalid image")
    with pytest.raises(ValueError, match="无法读取图片"):
        MODULE.collect_samples(raw_data, 2)


def test_augmentation_reports_image_write_failure(raw_data, tmp_path, monkeypatch):
    images, labels = tmp_path / "images", tmp_path / "labels"
    images.mkdir()
    labels.mkdir()
    monkeypatch.setattr(MODULE.cv2, "imwrite", lambda *args: False)
    with pytest.raises(OSError, match="写入图片失败"):
        MODULE.write_augmented_sample(raw_data / "image" / "0.jpg", raw_data / "label" / "0.txt", images, labels)


def test_invalid_input_keeps_existing_output(raw_data, tmp_path):
    (raw_data / "label" / "0.txt").write_text("7 0.1 0.2 0.3 0.4\n")
    output = tmp_path / "dataset"
    output.mkdir()
    (output / "keep.txt").write_text("existing data")
    result = subprocess.run([sys.executable, str(MODULE_PATH), "--new-data-root", str(raw_data),
                             "--output-root", str(output)], capture_output=True, text=True)
    assert result.returncode != 0
    assert "类别 ID 7" in result.stderr
    assert (output / "keep.txt").read_text() == "existing data"


@pytest.mark.parametrize("relation", ["same", "parent", "child"])
def test_cli_rejects_overlapping_paths(raw_data, relation):
    output = {"same": raw_data, "parent": raw_data.parent, "child": raw_data / "output"}[relation]
    result = subprocess.run([sys.executable, str(MODULE_PATH), "--new-data-root", str(raw_data),
                             "--output-root", str(output)], capture_output=True, text=True)
    assert result.returncode != 0
    assert "互相包含" in result.stderr
    assert (raw_data / "label" / "0.txt").exists()
