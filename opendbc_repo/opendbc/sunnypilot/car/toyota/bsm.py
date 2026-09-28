"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, CanData, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

# Enhanced BSM: blind-spot status for the 2017-20 Lexus IS, whose blind spot monitor sensors do not broadcast it.
# Their 0x3F6 (BSM) carries only the two "enabled" flags: 80 80 00 00 00 00 00 00 at 1 Hz on a whole logged route with
# numerous detections on both sides. openpilot reads 0x3F6 only on TSS2 cars.
#
# The sensors answer diagnostic requests on 0x750 (sub-addressed; left 0x41, right 0x42), replies on 0x758. The
# requests and the detection rule are dragonpilot's Enhanced BSM (arne182, rav4kumar), which a sunnypilot fork of 0.9.8
# carried for this car:
#   - a sensor is put in session 0x60 (41 02 10 60 ...), then read local identifier 0x69 (41 02 21 69 ...) every 20
#     control frames, left and right 10 frames apart;
#   - a side is detected when byte 5 or byte 6 of the positive reply (41 xx 61 69 ...) is above 10.
# The meaning of bytes 4-7 is not documented. Only positive single-frame replies from 0x41/0x42 count.
#
# Sessions are open only while driving: from 10 mph, above which the factory system reports vehicles (opendbc's notes
# on 0x3F6, from TSS2 cars), and closed again (41 02 10 01 ...) below 8 mph. Below that nothing is polled and the
# sensors are left to their factory functions at low speed, such as rear cross traffic alert on cars that have it. Nothing is sent in the first
# 2 s after the controls start. A side with no positive reply for 1 s reports clear; its session request is then
# repeated every 2 s while the car is above the closing speed. The panda allows exactly these six requests, with
# ToyotaSafetyFlagsSP.ENHANCED_BSM (safety/modes/toyota.h).
#
# Enabled in interface.py for the Lexus IS when the left sensor, the one on the CAN bus, answered the tester-present
# query at startup (Ecu.cornerRadar at (0x750, 0x41), values.py FW_QUERY_CONFIG).

BSM_DIAG_MSG = "BSM_DIAG_RESPONSE"
BSM_REQUEST_ADDR = 0x750
BSM_LEFT = 0x41
BSM_RIGHT = 0x42
BSM_SENSORS = (BSM_LEFT, BSM_RIGHT)

BSM_SESSION_REQUEST = b"\x02\x10\x60\x00\x00\x00\x00"   # diagnostic session control, session 0x60
BSM_DEFAULT_SESSION_REQUEST = b"\x02\x10\x01\x00\x00\x00\x00"   # back to the default session
BSM_POLL_REQUEST = b"\x02\x21\x69\x00\x00\x00\x00"      # read data by local identifier 0x69
BSM_POSITIVE_SID = 0x61
BSM_LOCAL_ID = 0x69
BSM_DETECT_THRESHOLD = 10

BSM_START_FRAME = round(2.0 / DT_CTRL)
BSM_POLL_PERIOD = 20
BSM_POLL_PHASE = {BSM_LEFT: 0, BSM_RIGHT: BSM_POLL_PERIOD // 2}
BSM_TIMEOUT_FRAMES = round(1.0 / DT_CTRL)
BSM_SESSION_RETRY_FRAMES = round(2.0 / DT_CTRL)
BSM_OPEN_SPEED = 10 * CV.MPH_TO_MS
BSM_CLOSE_SPEED = 8 * CV.MPH_TO_MS


def bsm_request(sub_addr: int, request: bytes) -> CanData:
  return CanData(BSM_REQUEST_ADDR, bytes([sub_addr]) + request, 0)


def bsm_detected(data_5: int, data_6: int) -> bool:
  return data_5 > BSM_DETECT_THRESHOLD or data_6 > BSM_DETECT_THRESHOLD


class BsmCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.enhanced_bsm = bool(CP_SP.flags & ToyotaFlagsSP.ENHANCED_BSM)
    self.bsm_frame = 0
    self.bsm_detected = {s: False for s in BSM_SENSORS}
    self.bsm_reply_frame: dict[int, int | None] = {s: None for s in BSM_SENSORS}   # last positive reply

  def bsm_fresh(self, sensor: int) -> bool:
    reply = self.bsm_reply_frame[sensor]
    return reply is not None and self.bsm_frame - reply < BSM_TIMEOUT_FRAMES

  def update_bsm(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    if not self.enhanced_bsm:
      return

    self.bsm_frame += 1
    replies = can_parsers[Bus.pt].vl_all[BSM_DIAG_MSG]
    for sub_addr, pci, sid, local_id, data_5, data_6 in zip(replies["SUB_ADDRESS"], replies["PCI"], replies["SID"],
                                                            replies["LOCAL_ID"], replies["DATA_5"], replies["DATA_6"],
                                                            strict=True):
      sensor = int(sub_addr)
      # positive single-frame reply to 0x21 0x69, long enough to carry bytes 5 and 6
      if sensor in BSM_SENSORS and int(sid) == BSM_POSITIVE_SID and int(local_id) == BSM_LOCAL_ID and 5 <= int(pci) <= 6:
        self.bsm_detected[sensor] = bsm_detected(int(data_5), int(data_6))
        self.bsm_reply_frame[sensor] = self.bsm_frame

    # reported only in the range the sensors are polled in (replies can only come from another tester below it)
    monitored = ret.vEgo >= BSM_CLOSE_SPEED
    ret.leftBlindspot = monitored and self.bsm_fresh(BSM_LEFT) and self.bsm_detected[BSM_LEFT]
    ret.rightBlindspot = monitored and self.bsm_fresh(BSM_RIGHT) and self.bsm_detected[BSM_RIGHT]


class BsmCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.enhanced_bsm = bool(CP_SP.flags & ToyotaFlagsSP.ENHANCED_BSM)
    self.bsm_driving = False                                                        # between opening and closing speed
    self.bsm_session_frame: dict[int, int | None] = {s: None for s in BSM_SENSORS}   # last session request, None = closed

  def create_bsm_msgs(self, CS, frame: int) -> list[CanData]:
    if not self.enhanced_bsm or frame < BSM_START_FRAME:
      return []

    v_ego = CS.out.vEgo
    if not self.bsm_driving and v_ego >= BSM_OPEN_SPEED:
      self.bsm_driving = True
    elif self.bsm_driving and v_ego < BSM_CLOSE_SPEED:
      self.bsm_driving = False

    msgs = []
    for sensor in BSM_SENSORS:
      if frame % BSM_POLL_PERIOD != BSM_POLL_PHASE[sensor]:
        continue
      last_session = self.bsm_session_frame[sensor]
      if not self.bsm_driving:
        if last_session is not None:
          msgs.append(bsm_request(sensor, BSM_DEFAULT_SESSION_REQUEST))
          self.bsm_session_frame[sensor] = None
      elif last_session is None or (not CS.bsm_fresh(sensor) and frame - last_session >= BSM_SESSION_RETRY_FRAMES):
        # open, or no positive reply for 1 s: (re)open the session in place of this poll
        msgs.append(bsm_request(sensor, BSM_SESSION_REQUEST))
        self.bsm_session_frame[sensor] = frame
      else:
        msgs.append(bsm_request(sensor, BSM_POLL_REQUEST))
    return msgs
