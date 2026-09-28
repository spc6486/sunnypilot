import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, CanData, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import CarParams
from opendbc.car.toyota.values import CAR, DBC, FW_QUERY_CONFIG, Ecu
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.toyota.bsm import BSM_CLOSE_SPEED, BSM_DIAG_MSG, BSM_LEFT, BSM_OPEN_SPEED, BSM_POLL_PERIOD, \
                                             BSM_RIGHT, BSM_SESSION_RETRY_FRAMES, BSM_START_FRAME, BSM_TIMEOUT_FRAMES, \
                                             BsmCarController, BsmCarState, bsm_detected
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP, TSS2_EPS_DBC

PT_DBC = DBC[CAR.LEXUS_IS][Bus.pt]
REPLY_ADDR = 0x758

# the six requests the panda allows (safety/modes/toyota.h)
SESSION = {BSM_LEFT: bytes.fromhex("4102106000000000"), BSM_RIGHT: bytes.fromhex("4202106000000000")}
POLL = {BSM_LEFT: bytes.fromhex("4102216900000000"), BSM_RIGHT: bytes.fromhex("4202216900000000")}
CLOSE = {BSM_LEFT: bytes.fromhex("4102100100000000"), BSM_RIGHT: bytes.fromhex("4202100100000000")}
DRIVING = 30 * CV.MPH_TO_MS


def reply(sensor: int, data_5: int, data_6: int, pci: int = 6, sid: int = 0x61, local_id: int = 0x69) -> CanData:
  return CanData(REPLY_ADDR, bytes([sensor, pci, sid, local_id, 0, data_5, data_6, 0]), 0)


def bsm_cp(enhanced=True):
  CP = structs.CarParams()
  CP_SP = structs.CarParamsSP()
  if enhanced:
    CP_SP.flags = int(ToyotaFlagsSP.ENHANCED_BSM)
  return CP, CP_SP


def cornerRadar_fw(sub_addr: int) -> CarParams.CarFw:
  fw = CarParams.CarFw()
  fw.ecu = Ecu.cornerRadar
  fw.address = 0x750
  fw.subAddress = sub_addr
  fw.fwVersion = b""
  fw.logging = True
  return fw


class BsmStateHarness:
  """BsmCarState with a real parser on the Lexus IS DBC"""
  def __init__(self, dbc=PT_DBC):
    self.cs = BsmCarState(*bsm_cp())
    self.cp = CANParser(dbc, [(BSM_DIAG_MSG, float('nan'))], 0)
    self.nanos = 0

  def step(self, frames=(), v_ego=DRIVING) -> structs.CarState:
    self.nanos += round(DT_CTRL * 1e9)
    self.cp.update([(self.nanos, list(frames))])
    ret = structs.CarState(vEgo=v_ego)
    self.cs.update_bsm(ret, {Bus.pt: self.cp})
    return ret


class TestBsmState(unittest.TestCase):
  def test_rule(self):
    self.assertFalse(bsm_detected(10, 10))
    self.assertTrue(bsm_detected(11, 0))
    self.assertTrue(bsm_detected(0, 11))

  def test_both_sides(self):
    h = BsmStateHarness()
    ret = h.step([reply(BSM_LEFT, 30, 0), reply(BSM_RIGHT, 0, 0)])
    self.assertEqual((ret.leftBlindspot, ret.rightBlindspot), (True, False))
    ret = h.step([reply(BSM_RIGHT, 0, 12)])
    self.assertEqual((ret.leftBlindspot, ret.rightBlindspot), (True, True))
    ret = h.step([reply(BSM_LEFT, 10, 10)])
    self.assertEqual((ret.leftBlindspot, ret.rightBlindspot), (False, True))

  def test_ignored_frames(self):
    for frame in (reply(BSM_LEFT, 30, 30, sid=0x7F),         # negative response
                  reply(BSM_LEFT, 30, 30, sid=0x50),         # reply to the session request
                  reply(BSM_LEFT, 30, 30, local_id=0x68),    # another local identifier
                  reply(0x0F, 30, 30),                        # another ECU (the radar's sub-address)
                  reply(BSM_LEFT, 30, 30, pci=0x10),         # first frame of a multi-frame reply
                  reply(BSM_LEFT, 30, 30, pci=4)):            # too short to carry bytes 5 and 6
      h = BsmStateHarness()
      ret = h.step([frame])
      self.assertFalse(ret.leftBlindspot or ret.rightBlindspot, frame.dat.hex())
      self.assertIsNone(h.cs.bsm_reply_frame[BSM_LEFT])

  def test_timeout(self):
    h = BsmStateHarness()
    self.assertTrue(h.step([reply(BSM_LEFT, 30, 30)]).leftBlindspot)
    for _ in range(BSM_TIMEOUT_FRAMES - 1):
      self.assertTrue(h.step().leftBlindspot)
    self.assertFalse(h.step().leftBlindspot)
    # no re-arming from the last frame the parser holds (the staging-c3 implementation kept a silent side detected)
    for _ in range(5 * BSM_TIMEOUT_FRAMES):
      self.assertFalse(h.step().leftBlindspot)
    self.assertTrue(h.step([reply(BSM_LEFT, 30, 30)]).leftBlindspot)

  def test_speed(self):
    h = BsmStateHarness()
    self.assertFalse(h.step([reply(BSM_LEFT, 30, 30)], v_ego=BSM_CLOSE_SPEED - 0.01).leftBlindspot)
    self.assertTrue(h.step(v_ego=BSM_CLOSE_SPEED + 0.01).leftBlindspot)

  def test_not_enhanced(self):
    cs = BsmCarState(*bsm_cp(enhanced=False))
    ret = structs.CarState()
    cs.update_bsm(ret, {})   # the parser has no BSM_DIAG_RESPONSE without the flag; it is never read
    self.assertFalse(ret.leftBlindspot or ret.rightBlindspot)

  def test_both_eps_dbcs(self):
    for dbc in (PT_DBC, TSS2_EPS_DBC[PT_DBC]):
      h = BsmStateHarness(dbc)
      self.assertTrue(h.step([reply(BSM_RIGHT, 0, 40)]).rightBlindspot, dbc)


class TestBsmController(unittest.TestCase):
  def run_frames(self, frames, fresh=lambda frame, sensor: False, speed=lambda frame: DRIVING):
    cc = BsmCarController(*bsm_cp())
    sends = {}
    for frame in range(frames):
      CS = SimpleNamespace(out=SimpleNamespace(vEgo=speed(frame)), bsm_fresh=lambda sensor, f=frame: fresh(f, sensor))
      msgs = cc.create_bsm_msgs(CS, frame)
      if msgs:
        sends[frame] = [(m.address, m.dat, m.src) for m in msgs]
    return sends

  def test_schedule(self):
    sends = self.run_frames(BSM_START_FRAME + 5 * BSM_POLL_PERIOD, fresh=lambda frame, sensor: True)
    self.assertNotIn(BSM_START_FRAME - 1, sends)
    self.assertEqual(sends[BSM_START_FRAME], [(0x750, SESSION[BSM_LEFT], 0)])
    self.assertEqual(sends[BSM_START_FRAME + 10], [(0x750, SESSION[BSM_RIGHT], 0)])
    for k in range(1, 5):
      self.assertEqual(sends[BSM_START_FRAME + k * BSM_POLL_PERIOD], [(0x750, POLL[BSM_LEFT], 0)])
      self.assertEqual(sends[BSM_START_FRAME + k * BSM_POLL_PERIOD + 10], [(0x750, POLL[BSM_RIGHT], 0)])
    self.assertEqual(len(sends), 10)   # one request per sensor per poll period

  def test_session_retry(self):
    # the left sensor never answers, the right one does: only the left session request repeats, every 2 s
    sends = self.run_frames(BSM_START_FRAME + 3 * BSM_SESSION_RETRY_FRAMES + 1,
                            fresh=lambda frame, sensor: sensor == BSM_RIGHT)
    left_sessions = [f for f, m in sends.items() if (0x750, SESSION[BSM_LEFT], 0) in m]
    right_sessions = [f for f, m in sends.items() if (0x750, SESSION[BSM_RIGHT], 0) in m]
    self.assertEqual(left_sessions, [BSM_START_FRAME + k * BSM_SESSION_RETRY_FRAMES for k in range(4)])
    self.assertEqual(right_sessions, [BSM_START_FRAME + 10])

  def test_speed_gate(self):
    # parked, then 10 mph from 4 s, slowing through the hysteresis band from 8 s, below 8 mph from 12 s, 10 mph from 16 s
    def speed(frame):
      t = frame * DT_CTRL
      if t < 4:
        return 0.
      if t < 8:
        return BSM_OPEN_SPEED
      if t < 12:
        return (BSM_OPEN_SPEED + BSM_CLOSE_SPEED) / 2
      if t < 16:
        return BSM_CLOSE_SPEED - 0.01
      return BSM_OPEN_SPEED

    sends = self.run_frames(round(20 / DT_CTRL), fresh=lambda frame, sensor: True, speed=speed)
    events = [(f, dat) for f, msgs in sends.items() for _, dat, _ in msgs if dat not in POLL.values()]
    self.assertEqual(events, [(400, SESSION[BSM_LEFT]), (410, SESSION[BSM_RIGHT]),     # opened at 10 mph
                              (1200, CLOSE[BSM_LEFT]), (1210, CLOSE[BSM_RIGHT]),       # closed below 8 mph, not at 9
                              (1600, SESSION[BSM_LEFT]), (1610, SESSION[BSM_RIGHT])])  # and opened again
    polls = sorted(f for f, msgs in sends.items() for _, dat, _ in msgs if dat in POLL.values())
    self.assertTrue(all(400 < f < 1200 or f > 1610 for f in polls))
    self.assertEqual(len([f for f in polls if 400 < f < 1200]), 2 * (800 // BSM_POLL_PERIOD) - 2)

  def test_never_fast_enough(self):
    self.assertEqual(self.run_frames(round(30 / DT_CTRL), speed=lambda frame: BSM_OPEN_SPEED - 0.01), {})

  def test_not_enhanced(self):
    cc = BsmCarController(*bsm_cp(enhanced=False))
    self.assertFalse(any(cc.create_bsm_msgs(None, f) for f in range(3 * BSM_START_FRAME)))

  def test_panda_allows_exactly_these(self):
    # every request passes the panda with the flags interface.py sets, none without them
    def speed(frame):
      return DRIVING if (frame // 1000) % 2 == 0 else 0.

    requests = {dat for msgs in self.run_frames(round(40 / DT_CTRL), speed=speed).values() for _, dat, _ in msgs}
    self.assertEqual(requests, set(SESSION.values()) | set(POLL.values()) | set(CLOSE.values()))
    safety = libsafety_py.libsafety
    for sp, allowed in ((ToyotaSafetyFlagsSP.UNSUPPORTED_DSU | ToyotaSafetyFlagsSP.ENHANCED_BSM, True),
                        (ToyotaSafetyFlagsSP.UNSUPPORTED_DSU, False)):
      for stock_long in (False, True):
        safety.set_current_safety_param_sp(sp)
        param = 77 | (2 << 8 if stock_long else 0)   # Lexus IS EPS factor, TOYOTA_PARAM_STOCK_LONGITUDINAL
        safety.set_safety_hooks(CarParams.SafetyModel.toyota, param)
        safety.init_tests()
        for dat in requests:
          self.assertEqual(allowed, safety.safety_tx_hook(libsafety_py.make_CANPacket(0x750, 0, dat)), (sp, stock_long, dat.hex()))


class TestBsmInterface(unittest.TestCase):
  """Detection at startup and the path through the real Lexus IS CarInterface"""
  def interface(self, car_fw, candidate=CAR.LEXUS_IS, smart_dsu=True, eps_len=None):
    fingerprint = {0: {0x2FF: 8, 0x3F6: 8} if smart_dsu else {0x3F6: 8}, 1: {}, 2: {}}
    if eps_len is not None:
      fingerprint[0][0x262] = eps_len
    CarInterface = interfaces[candidate]
    CP = CarInterface.get_params(candidate, fingerprint, car_fw, alpha_long=smart_dsu, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, car_fw, alpha_long=smart_dsu, is_release_sp=False, docs=False)
    return CarInterface(CP, CP_SP)

  def test_fw_query(self):
    requests = [r for r in FW_QUERY_CONFIG.requests if Ecu.cornerRadar in r.whitelist_ecus]
    self.assertEqual(len(requests), 1)
    self.assertTrue(requests[0].logging)
    self.assertEqual(requests[0].request, [b"\x3e"])
    self.assertEqual(requests[0].bus, 0)
    self.assertEqual({(e[1], e[2]) for e in FW_QUERY_CONFIG.extra_ecus if e[0] == Ecu.cornerRadar}, {(0x750, 0x41)})

  def test_detection(self):
    for candidate, car_fw, expected in ((CAR.LEXUS_IS, [], False),
                                        (CAR.LEXUS_IS, [cornerRadar_fw(0x41)], True),
                                        (CAR.LEXUS_IS, [cornerRadar_fw(0x42)], False),     # only the left sensor is queried
                                        (CAR.LEXUS_IS, [cornerRadar_fw(0x43)], False),
                                        (CAR.LEXUS_RC, [cornerRadar_fw(0x41)], False),      # sub-addresses not known there
                                        (CAR.LEXUS_IS_TSS2, [cornerRadar_fw(0x41)], False)):
      for smart_dsu in (True, False):
        with self.subTest(candidate=candidate, fw=[hex(f.subAddress) for f in car_fw], smart_dsu=smart_dsu):
          ci = self.interface(car_fw, candidate, smart_dsu)
          self.assertEqual(bool(ci.CP_SP.flags & ToyotaFlagsSP.ENHANCED_BSM), expected)
          self.assertEqual(bool(ci.CP_SP.safetyParam & ToyotaSafetyFlagsSP.ENHANCED_BSM), expected)
          if candidate == CAR.LEXUS_IS:
            self.assertEqual(ci.CP.enableBsm, expected)
          self.assertEqual(REPLY_ADDR in ci.can_parsers[Bus.pt].addresses, expected)

  def run_frames(self, ci, frames, left=0, speed_kph=50.):
    CC = structs.CarControl().as_reader()
    CC_SP = structs.CarControlSP()
    packer = CANPacker(ci.can_parsers[Bus.pt].dbc_name)
    wheels = {f"WHEEL_SPEED_{w}": speed_kph for w in ("FL", "FR", "RL", "RR")}
    sends, states = [], []
    for i in range(frames):
      nanos = round((i + 1) * DT_CTRL * 1e9)
      cans = [CanData(*packer.make_can_msg("WHEEL_SPEEDS", 0, wheels))]
      if i >= BSM_START_FRAME and i % BSM_POLL_PERIOD == 1:
        cans.append(reply(BSM_LEFT, left, 0))
      states.append(ci.update([(nanos, cans)])[0])
      sends.append([CanData(*m) for m in ci.apply(CC, CC_SP, nanos)[1]])
    return sends, states

  def test_through_interface(self):
    for smart_dsu in (True, False):          # openpilot longitudinal with the C-SDSU, and stock longitudinal
      for eps_len in (None, 5, 8):
        with self.subTest(smart_dsu=smart_dsu, eps_len=eps_len):
          ci = self.interface([cornerRadar_fw(0x41)], smart_dsu=smart_dsu, eps_len=eps_len)
          self.assertEqual(ci.CP.openpilotLongitudinalControl, smart_dsu)
          self.assertTrue(ci.can_parsers[Bus.pt].message_states[REPLY_ADDR].ignore_alive)   # no CAN error without replies
          sends, states = self.run_frames(ci, BSM_START_FRAME + 60, left=40)
          self.assertGreater(states[-1].vEgo, BSM_OPEN_SPEED)
          bsm = [(i, m.dat) for i, s in enumerate(sends) for m in s if m.address == 0x750]
          self.assertEqual(bsm[:3], [(BSM_START_FRAME, SESSION[BSM_LEFT]), (BSM_START_FRAME + 10, SESSION[BSM_RIGHT]),
                                     (BSM_START_FRAME + 20, POLL[BSM_LEFT])])
          self.assertTrue(states[-1].leftBlindspot)
          self.assertFalse(states[-1].rightBlindspot)
          self.assertTrue(all(m.src == 0 for s in sends for m in s if m.address == 0x750))

  def test_parked(self):
    ci = self.interface([cornerRadar_fw(0x41)])
    sends, states = self.run_frames(ci, BSM_START_FRAME + 200, left=40, speed_kph=0.)
    self.assertFalse(any(m.address == 0x750 for s in sends for m in s))
    self.assertFalse(any(s.leftBlindspot for s in states))   # below the closing speed a reply (another tester's) is not reported

  def test_without_sensor(self):
    ci = self.interface([])
    sends, states = self.run_frames(ci, BSM_START_FRAME + 60, left=40)
    self.assertFalse(any(m.address == 0x750 for s in sends for m in s))
    self.assertFalse(any(s.leftBlindspot for s in states))


if __name__ == "__main__":
  unittest.main()
