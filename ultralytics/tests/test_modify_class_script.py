import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / 'tools' / 'modify_class.py'
SPEC = importlib.util.spec_from_file_location('modify_class', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_merge_classes_preserves_boxes_and_is_repeatable(tmp_path):
    classes = tmp_path / 'classes.txt'
    classes.write_text('community\ncommunitydda\nnon-community\n', encoding='utf-8')
    label = tmp_path / 'sample.txt'
    label.write_bytes(b'0 0.1 0.2 0.3 0.4\r\n1 0.2 0.3 0.4 0.5\r\n\r\n2 0.3 0.4 0.5 0.6')
    (tmp_path / 'empty.txt').write_text('')
    (tmp_path / 'nested.txt').mkdir()

    MODULE.modify_yolo_labels(tmp_path)

    expected = b'0 0.1 0.2 0.3 0.4\r\n0 0.2 0.3 0.4 0.5\r\n\r\n1 0.3 0.4 0.5 0.6'
    assert label.read_bytes() == expected
    assert classes.read_text() == 'community\nnon-community\n'
    assert (tmp_path / 'empty.txt').read_bytes() == b''
    MODULE.modify_yolo_labels(tmp_path)
    assert label.read_bytes() == expected


def test_mapping_uses_class_names_instead_of_fixed_ids(tmp_path):
    (tmp_path / 'classes.txt').write_text('non-community\ncommunitydda\ncommunity\n')
    label = tmp_path / 'sample.txt'
    label.write_text('0 0.1 0.2 0.3 0.4\n2 0.2 0.3 0.4 0.5\n')

    MODULE.modify_yolo_labels(tmp_path)

    assert label.read_text() == '1 0.1 0.2 0.3 0.4\n0 0.2 0.3 0.4 0.5\n'


@pytest.mark.parametrize('invalid_id', ['3', '-1', 'invalid'])
def test_invalid_labels_fail_before_any_write(tmp_path, invalid_id):
    classes = tmp_path / 'classes.txt'
    class_text = 'community\ncommunitydda\nnon-community\n'
    classes.write_text(class_text)
    valid = tmp_path / 'a.txt'
    valid_text = '1 0.1 0.2 0.3 0.4\n'
    valid.write_text(valid_text)
    (tmp_path / 'z.txt').write_text(f'{invalid_id} 0.1 0.2 0.3 0.4\n')

    with pytest.raises(ValueError):
        MODULE.modify_yolo_labels(tmp_path)

    assert classes.read_text() == class_text
    assert valid.read_text() == valid_text


@pytest.mark.parametrize('class_text', ['', 'community\nother\n'])
def test_rejects_empty_or_unknown_classes(tmp_path, class_text):
    classes = tmp_path / 'classes.txt'
    classes.write_text(class_text)

    with pytest.raises(ValueError):
        MODULE.modify_yolo_labels(tmp_path)

    assert classes.read_text() == class_text


def test_legacy_target_class_keeps_class_list(tmp_path):
    classes = tmp_path / 'classes.txt'
    classes.write_text('community\nnon-community\n')
    label = tmp_path / 'sample.txt'
    label.write_text('0 0.1 0.2 0.3 0.4\n\n1 0.2 0.3 0.4 0.5\n')

    MODULE.modify_yolo_labels(tmp_path, 2)

    assert label.read_text() == '2 0.1 0.2 0.3 0.4\n\n2 0.2 0.3 0.4 0.5\n'
    assert classes.read_text() == 'community\nnon-community\n'


def test_missing_directory_and_negative_target(tmp_path):
    with pytest.raises(ValueError, match='不存在'):
        MODULE.modify_yolo_labels(tmp_path / 'missing')
    with pytest.raises(ValueError, match='负数'):
        MODULE.modify_yolo_labels(tmp_path, -1)
    with pytest.raises(FileNotFoundError):
        MODULE.modify_yolo_labels(tmp_path)


def test_cli_merges_classes_and_reports_errors(tmp_path):
    (tmp_path / 'classes.txt').write_text('community\ncommunitydda\nnon-community\n')
    label = tmp_path / 'sample.txt'
    label.write_text('1 0.1 0.2 0.3 0.4\n2 0.2 0.3 0.4 0.5\n')

    result = subprocess.run([sys.executable, str(MODULE_PATH), '--input', str(tmp_path)], capture_output=True)

    assert result.returncode == 0
    assert label.read_text() == '0 0.1 0.2 0.3 0.4\n1 0.2 0.3 0.4 0.5\n'
    result = subprocess.run([sys.executable, str(MODULE_PATH), '--input', str(tmp_path / 'missing')], capture_output=True)
    assert result.returncode == 1
    assert '错误' in result.stderr.decode()
