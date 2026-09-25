"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum
from collections import namedtuple

from opendbc.car import Bus, structs
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser

MadsDataSP = namedtuple("MadsDataSP", ["enable_mads", "mads_enabled", "paused", "lat_active"])

MadsState = structs.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState

# LDA button -> MADS on TSS-P cars whose LDA switch is wired to the DSU (Lexus IS/RC 2017-19, UNSUPPORTED_DSU).
# The camera's LKAS_HUD (0x412, bus 2, no checksum) is sent at ~1 Hz and additionally at once when its content
# changes, so a button press is reported by an immediate frame. LKAS_STATUS: 0 = LDA off, 1 = on, 2 = steering
# assist controlling, 3 = departure alert (2 and 3 are transient states while on, not presses).
# A frame is accepted as an LDA level only if all of byte 0 agrees, which is what the camera sends (568/568 logged
# frames): off = LKAS_STATUS, LEFT_LINE and RIGHT_LINE all 0; on = all three non-zero. Flipping between the two needs
# at least 3 consistent bit changes, so a corrupted frame is ignored (level held) instead of toggling MADS, and the
# press is acted on from the first frame -- no second-frame confirmation delay.


def lda_level(lkas_hud: dict) -> bool | None:
  status, left, right = lkas_hud["LKAS_STATUS"], lkas_hud["LEFT_LINE"], lkas_hud["RIGHT_LINE"]
  if status == 0 and left == 0 and right == 0:
    return False
  if status != 0 and left != 0 and right != 0:
    return True
  return None


class MadsCarController:
  def __init__(self):
    self.mads = MadsDataSP(False, False, False, False)
    self.lkas_status_last: int | None = None   # last indicator value sent, to push changes without waiting

  @staticmethod
  def mads_status_update(CC: structs.CarControl, CC_SP: structs.CarControlSP) -> MadsDataSP:
    return MadsDataSP(CC_SP.mads.available, CC_SP.mads.enabled, CC_SP.mads.state == MadsState.paused, CC.latActive)

  def lkas_status(self) -> int | None:
    """Cluster LKA indicator driven like the factory LDA indicator:
         2 green  - openpilot is steering
         1 white  - MADS enabled but not steering right now (standstill, steer fault, ...)
         0 off    - MADS disabled (cruise MAIN off) or paused (LDA button, P/R/N), i.e. it will not steer until
                    resumed -- the factory button likewise turns the indicator off
       None keeps the stock always-white behaviour when MADS is not available."""
    if not self.mads.enable_mads:
      return None
    if self.mads.lat_active:
      return 2
    if self.mads.paused or not self.mads.mads_enabled:
      return 0
    return 1

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    self.mads = self.mads_status_update(CC, CC_SP)


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.lkas_status_mads = bool(CP.flags & ToyotaFlags.UNSUPPORTED_DSU)

    self.lda_on = False               # accepted LDA level
    self.lda_on_init = False          # first valid camera frame seeds lda_on without generating a press
    self.lkas_hud_ts = 0
    self.lkas_button_edge = False     # True for the one CarState cycle in which a press was accepted

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    self.lkas_button_edge = False
    if not self.lkas_status_mads:
      return

    cp_cam = can_parsers[Bus.cam]
    ts = cp_cam.ts_nanos["LKAS_HUD"]["LKAS_STATUS"]
    if ts == 0 or ts == self.lkas_hud_ts:
      return                          # no new camera frame this cycle (ts == 0: parser defaults, not a real frame)
    self.lkas_hud_ts = ts

    level = lda_level(cp_cam.vl["LKAS_HUD"])
    if level is None:
      return                          # inconsistent frame: hold the last accepted level

    if not self.lda_on_init:
      self.lda_on = level
      self.lda_on_init = True
    elif level != self.lda_on:
      self.lda_on = level
      self.lkas_button_edge = True
