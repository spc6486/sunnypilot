import json

import pytest

from openpilot.sunnypilot.lexus_is import features


@pytest.fixture(autouse=True)
def fresh_cache():
  features._cache = None
  yield
  features._cache = None


def write(tmp_path, content):
  p = tmp_path / "features.json"
  p.write_text(content if isinstance(content, str) else json.dumps(content))
  return str(p)


def test_missing_file_is_defaults(tmp_path):
  assert features.load(str(tmp_path / "absent.json")) == features.DEFAULTS


def test_defaults_are_all_on():
  assert all(features.DEFAULTS.values())


def test_override_known_keys(tmp_path):
  s = features.load(write(tmp_path, {"rsa": False, "set_speed_law": False}))
  assert s == {**features.DEFAULTS, "rsa": False, "set_speed_law": False}


@pytest.mark.parametrize("content", ["{not json", "[1, 2]", '"text"'])
def test_bad_file_is_defaults(tmp_path, content):
  assert features.load(write(tmp_path, content)) == features.DEFAULTS


def test_unknown_and_non_bool_ignored(tmp_path):
  s = features.load(write(tmp_path, {"rsa": 0, "enhanced_bsm": "false", "unknown": False}))
  assert s == features.DEFAULTS


def test_opendbc_params_list(tmp_path, monkeypatch):
  monkeypatch.setattr(features, "FEATURES_PATH", write(tmp_path, {"enhanced_bsm": False}))
  assert features.opendbc_params_list() == [{"LexusIsLdaMads": 1}, {"LexusIsRsa": 1}, {"LexusIsEnhancedBsm": 0}]


def test_read_once(tmp_path, monkeypatch):
  path = write(tmp_path, {"rsa": False})
  monkeypatch.setattr(features, "FEATURES_PATH", path)
  assert not features.enabled("rsa")
  write(tmp_path, {"rsa": True})
  assert not features.enabled("rsa")  # cached until the next start
