"""
Lexus IS branch: per-feature settings.

The settings live in a JSON file on the device (/data/lexus_is_features.json), outside the repository, so they survive
branch updates. The file is read once per process; a change takes effect at the next start of openpilot. Every feature
defaults to on (the branch's behaviour before the settings existed), so a missing, unreadable or partial file changes
nothing. Example (any subset of the keys):

  {"set_speed_law": true, "fd_personality": true, "lda_mads": true, "rsa": true, "enhanced_bsm": false,
   "boot_recovery": true, "audio_ready_gate": true}

  set_speed_law   shaped set-speed law in the longitudinal planner (selfdrive/controls/lib/longitudinal_planner.py)
  fd_personality  the factory follow-distance selector sets the longitudinal personality (selfdrive/car/card.py)
  lda_mads        the LDA button pauses/resumes MADS; the cluster LKA indicator follows MADS (opendbc toyota mads.py)
  rsa             speed-limit sign on the cluster from the navigation limit (opendbc toyota rsa.py)
  enhanced_bsm    blind-spot status polled from the blind spot monitor sensors (opendbc toyota bsm.py)
  boot_recovery   restart micd / soundd / qcomgpsd after a crash, log why the audio stream did not open, retry the
                  modem diag port (sunnypilot/lexus_is/boot_recovery.py)
  audio_ready_gate  no engagement until soundd has its audio stream open (selfdrive/selfdrived/selfdrived.py)

The car-side features (lda_mads, rsa, enhanced_bsm) reach opendbc through the params list card.py passes to get_car
(sunnypilot/selfdrive/car/interfaces.py initialize_params -> opendbc setup_interfaces), where a feature that is off is
removed before the car interface and the panda safety config are built.
"""
import json
import os

from openpilot.common.swaglog import cloudlog

FEATURES_PATH = os.environ.get("LEXUS_IS_FEATURES", "/data/lexus_is_features.json")

DEFAULTS: dict[str, bool] = {
  "set_speed_law": True,
  "fd_personality": True,
  "lda_mads": True,
  "rsa": True,
  "enhanced_bsm": True,
  "boot_recovery": True,
  "audio_ready_gate": True,
}

# the names opendbc's setup_interfaces reads for the car-side features (sunnypilot/car/interfaces.py, _initialize_toyota)
OPENDBC_KEYS: dict[str, str] = {
  "lda_mads": "LexusIsLdaMads",
  "rsa": "LexusIsRsa",
  "enhanced_bsm": "LexusIsEnhancedBsm",
}

_cache: dict[str, bool] | None = None


def load(path: str | None = None) -> dict[str, bool]:
  """The settings: DEFAULTS overridden by the file's boolean values for known keys. Anything else is ignored."""
  settings = dict(DEFAULTS)
  path = path or FEATURES_PATH
  try:
    with open(path) as f:
      data = json.load(f)
  except FileNotFoundError:
    return settings
  except (OSError, ValueError) as e:
    cloudlog.warning(f"lexus_is features: {path} not read ({e}); using defaults")
    return settings
  if not isinstance(data, dict):
    cloudlog.warning(f"lexus_is features: {path} is not a JSON object; using defaults")
    return settings
  for key, value in data.items():
    if key in settings and isinstance(value, bool):
      settings[key] = value
    else:
      cloudlog.warning(f"lexus_is features: ignored {key!r}={value!r} (unknown key or not true/false)")
  return settings


def settings() -> dict[str, bool]:
  global _cache
  if _cache is None:
    _cache = load()
    cloudlog.info(f"lexus_is features: {_cache}")
  return _cache


def enabled(name: str) -> bool:
  return settings()[name]


def opendbc_params_list() -> list[dict[str, int]]:
  """The car-side settings in the form card.py passes to get_car (a list of one-key dicts)."""
  return [{key: int(enabled(name))} for name, key in OPENDBC_KEYS.items()]
