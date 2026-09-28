"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, CanData, DT_CTRL, structs
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.can.parser import CANParser

# Road-sign display (RSA) from the navigation speed limit, on the UNSUPPORTED_DSU cars (Lexus IS 2017-19, RC 2018-20,
# GS F 2016). The TSS-P camera of the one such car logged, a 2017 Lexus IS, recognizes no signs: its RSA1/RSA2
# (0x489/0x48A, bus 2, 1 Hz) are empty on every logged route. With openpilot longitudinal the panda blocks those two
# frames from the car and openpilot sends them instead, carrying the limit the head unit reports in NAV_SPEED_LIMIT
# (0x48C, bus 0, 2 Hz). On that car the gateway forwards 0x489/0x48A from the camera's bus to the cluster's bus, and
# not 0x48C. Not known: whether every camera on these platforms is like it; one that shows signs loses them here.
#
# NAV_SPEED_LIMIT byte 0 = 0x41 + limit / 5 with byte 1 = 1, or byte 0 = byte 1 = 0 when the map has no limit.
# A 2023 RC350's own camera read signs of 50, 55 and 65 mph alongside codes 0x4B, 0x4C and 0x4E in all 7 intervals
# of its route; the logged Lexus IS routes read 25-35 mph on city streets. The frame has no checksum. A new value is
# accepted once the next frame repeats it (0.5 s). In all 8 logged routes that carry the frame, every change of bytes
# 0-1 came with a new update counter in byte 7, so a value that changed while the counter did not needs a third
# identical frame (1 s): a single odd frame never reaches the sign, and a value the head unit keeps sending still does.
# Limits outside 5-85 mph and other byte-1 values (no other unit was seen) count as no limit.
#
# The frames reproduce the factory ones (2023 RC350 route) with the highlight bit clear: RSA1 = 24 00 LL 00 00 20 00 cc
# (sign type 36, the US speed-limit sign; LL = limit in mph) and RSA2 = 00 00 00 00 00 20 08 8c (one sign shown, mph).
# Without a sign: 00 00 00 00 00 00 00 0c and 00 00 00 00 00 00 00 8c, as the Lexus IS camera sends. cc = SYNCID
# 1-15, the same in both frames and advanced on every send, so no two consecutive sends carry the same SYNCID. Both
# frames go out together, at 1 Hz and at once when the sign changes. An accepted "no limit" blanks the sign only after
# 3 s, so that short gaps in the map data (2 s on a logged IS route) do not blink it; 2 s without a NAV_SPEED_LIMIT
# frame blanks it at once. The highlight, 0x36D and RSA3 are not sent here: the cluster translator on the car this was
# written for provides them.

NAV_MSG = "NAV_SPEED_LIMIT"
NAV_CODE_OFFSET = 0x41
NAV_LIMIT_MIN_MPH = 5
NAV_LIMIT_MAX_MPH = 85
NAV_REPEATS = 2                   # identical frames that accept a new value
NAV_REPEATS_SAME_COUNTER = 3      # the same, when the value changed without a new update counter
NAV_TIMEOUT_FRAMES = round(2.0 / DT_CTRL)
NAV_NONE_HOLD_FRAMES = round(3.0 / DT_CTRL)
RSA_PERIOD_FRAMES = round(1.0 / DT_CTRL)
RSA1_ADDR = 0x489
RSA2_ADDR = 0x48A
RSA_SIGN_US_SPEED_LIMIT = 0x24


def nav_limit_mph(code: int, flag: int) -> int | None:
  """NAV_SPEED_LIMIT bytes 0-1 -> posted limit in mph, or None for no limit, an unseen unit or an implausible value."""
  if flag != 1:
    return None
  limit = 5 * (code - NAV_CODE_OFFSET)
  return limit if NAV_LIMIT_MIN_MPH <= limit <= NAV_LIMIT_MAX_MPH else None


def rsa_frames(limit_mph: int | None, syncid: int) -> list[CanData]:
  if limit_mph is None:
    rsa1 = bytes([0, 0, 0, 0, 0, 0, 0, syncid])
    rsa2 = bytes([0, 0, 0, 0, 0, 0, 0, 0x80 | syncid])
  else:
    rsa1 = bytes([RSA_SIGN_US_SPEED_LIMIT, 0, limit_mph, 0, 0, 0x20, 0, syncid])
    rsa2 = bytes([0, 0, 0, 0, 0, 0x20, 0x08, 0x80 | syncid])
  return [CanData(RSA1_ADDR, rsa1, 0), CanData(RSA2_ADDR, rsa2, 0)]


class RsaCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.rsa_nav = bool(CP.flags & ToyotaFlags.UNSUPPORTED_DSU)
    self.nav_speed_limit: int | None = None   # accepted head-unit limit in mph, None = no limit
    self.rsa_limit: int | None = None         # limit on the sign: nav_speed_limit, with "no limit" held 3 s
    self.nav_ts = 0
    self.nav_age = 0
    self.nav_last: tuple[int, int, int] | None = None   # (code, flag, counter) of the last frame
    self.nav_candidate: int | None = None
    self.nav_repeats = 0
    self.nav_repeats_needed = NAV_REPEATS
    self.rsa_none_frames = 0

  def update_rsa(self, can_parsers: dict[StrEnum, CANParser]) -> None:
    if not self.rsa_nav:
      return

    if not self._update_nav(can_parsers[Bus.pt]):
      self.rsa_limit = None                   # the head unit went silent: no hold
      self.rsa_none_frames = 0
    elif self.nav_speed_limit is not None:
      self.rsa_limit = self.nav_speed_limit
      self.rsa_none_frames = 0
    elif self.rsa_limit is not None:
      self.rsa_none_frames += 1
      if self.rsa_none_frames >= NAV_NONE_HOLD_FRAMES:
        self.rsa_limit = None
        self.rsa_none_frames = 0

  def _update_nav(self, cp: CANParser) -> bool:
    """Take a new NAV_SPEED_LIMIT frame into nav_speed_limit; False once the message has been silent for 2 s."""
    ts = cp.ts_nanos[NAV_MSG]["SPEED_LIMIT_CODE"]
    if ts == 0 or ts == self.nav_ts:
      self.nav_age += 1
      if self.nav_age > NAV_TIMEOUT_FRAMES:
        self.nav_speed_limit = None
        self.nav_last = None
        self.nav_candidate = None
        self.nav_repeats = 0
        return False
      return True
    self.nav_ts = ts
    self.nav_age = 0

    msg = cp.vl[NAV_MSG]
    frame = (int(msg["SPEED_LIMIT_CODE"]), int(msg["SPEED_LIMIT_FLAG"]), int(msg["UPDATE_COUNTER"]))
    value = nav_limit_mph(frame[0], frame[1])
    if self.nav_repeats and value == self.nav_candidate:
      self.nav_repeats += 1
    else:
      same_counter = self.nav_last is not None and frame[:2] != self.nav_last[:2] and frame[2] == self.nav_last[2]
      self.nav_candidate = value
      self.nav_repeats = 1
      self.nav_repeats_needed = NAV_REPEATS_SAME_COUNTER if same_counter else NAV_REPEATS
    self.nav_last = frame
    if self.nav_repeats >= self.nav_repeats_needed:
      self.nav_speed_limit = value
    return True


class RsaCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    # the panda allows RSA1/RSA2 only with openpilot longitudinal on UNSUPPORTED_DSU cars without a gas interceptor
    self.rsa_tx = bool(CP.flags & ToyotaFlags.UNSUPPORTED_DSU) and CP.openpilotLongitudinalControl and \
                  not (CP.flags & ToyotaFlags.SECOC) and not CP_SP.enableGasInterceptor
    self.rsa_syncid = 0
    self.rsa_sent = False
    self.rsa_last: int | None = None

  def create_rsa_msgs(self, CS, frame: int) -> list[CanData]:
    if not self.rsa_tx:
      return []

    limit = CS.rsa_limit
    if self.rsa_sent and frame % RSA_PERIOD_FRAMES != 0 and limit == self.rsa_last:
      return []
    self.rsa_sent = True
    self.rsa_last = limit
    self.rsa_syncid = self.rsa_syncid % 15 + 1
    return rsa_frames(limit, self.rsa_syncid)
