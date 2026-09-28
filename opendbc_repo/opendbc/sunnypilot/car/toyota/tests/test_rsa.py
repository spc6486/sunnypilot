import unittest
from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.car import Bus, CanData, DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR, DBC, ToyotaFlags
from opendbc.sunnypilot.car.toyota.rsa import NAV_MSG, NAV_NONE_HOLD_FRAMES, NAV_REPEATS, NAV_REPEATS_SAME_COUNTER, \
                                             NAV_TIMEOUT_FRAMES, RSA1_ADDR, RSA2_ADDR, RSA_PERIOD_FRAMES, RsaCarController, \
                                             RsaCarState, nav_limit_mph, rsa_frames
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

PT_DBC = DBC[CAR.LEXUS_IS][Bus.pt]
NAV_ADDR = 0x48C
NAV_PERIOD_FRAMES = 50   # the head unit sends NAV_SPEED_LIMIT at 2 Hz

# RSA1 bytes 0-6 and RSA2 bytes 0-6 of the factory frames on a 2023 RC350 route (byte 7 = SYNCID, RSA2 | 0x80), for the
# frames with the highlight bit clear; the route's highlighted 55 mph frame (24 01 37 ...) differs only in byte 1
FACTORY_RSA1 = {50: "24003200002000", 55: "24003700002000", 65: "24004100002000", None: "00000000000000"}
FACTORY_RSA2 = {True: "00000000002008", False: "00000000000000"}


def car_params(flags=ToyotaFlags.UNSUPPORTED_DSU, op_long=True, interceptor=False):
  CP = structs.CarParams()
  CP.flags = int(flags)
  CP.openpilotLongitudinalControl = op_long
  CP_SP = structs.CarParamsSP()
  CP_SP.enableGasInterceptor = interceptor
  return CP, CP_SP


def nav_frame(code, flag, counter):
  return bytes([code, flag, 0, 0, 0, 0, 0, counter])


def cs_with(limit):
  return SimpleNamespace(rsa_limit=limit)


class NavFeed:
  """RsaCarState fed through a CANParser at the control rate"""
  def __init__(self):
    self.cp = CANParser(PT_DBC, [(NAV_MSG, float('nan'))], 0)
    self.cs = RsaCarState(*car_params())
    self.t = 0

  def step(self, dat=None):
    self.t += round(DT_CTRL * 1e9)
    self.cp.update([(self.t, [(NAV_ADDR, dat, 0)] if dat is not None else [])])
    self.cs.update_rsa({Bus.pt: self.cp})
    return self.cs.rsa_limit

  def frame(self, code, flag, counter):
    """one head-unit frame, then the control cycles until the next one; returns the sign after each cycle"""
    signs = [self.step(nav_frame(code, flag, counter))]
    for _ in range(NAV_PERIOD_FRAMES - 1):
      signs.append(self.step())
    return signs


class TestRsaFrames(unittest.TestCase):
  def test_spec_timings(self):
    # the translator spec: 1 Hz sends; 2 s silence blanks at once; "no limit" held 3 s; a new value needs the next frame
    # to repeat it, a value changed without a new update counter a third identical frame
    self.assertEqual(DT_CTRL, 0.01)
    self.assertEqual((RSA_PERIOD_FRAMES, NAV_TIMEOUT_FRAMES, NAV_NONE_HOLD_FRAMES), (100, 200, 300))
    self.assertEqual((NAV_REPEATS, NAV_REPEATS_SAME_COUNTER), (2, 3))

  def test_nav_limit_decode(self):
    self.assertEqual([nav_limit_mph(c, 1) for c in (0x46, 0x47, 0x48, 0x4B, 0x4C, 0x4E)], [25, 30, 35, 50, 55, 65])
    self.assertEqual((nav_limit_mph(0x42, 1), nav_limit_mph(0x52, 1)), (5, 85))
    for code, flag in ((0, 0), (0x46, 0), (0x46, 2), (0x41, 1), (0x40, 1), (0x53, 1), (0xFF, 1)):
      self.assertIsNone(nav_limit_mph(code, flag), (code, flag))

  def test_frames_match_factory(self):
    for syncid in (1, 7, 15):
      for limit, rsa1 in FACTORY_RSA1.items():
        f1, f2 = rsa_frames(limit, syncid)
        self.assertEqual((f1.address, f1.src, f2.address, f2.src), (RSA1_ADDR, 0, RSA2_ADDR, 0))
        self.assertEqual(f1.dat.hex(), rsa1 + f"{syncid:02x}")
        self.assertEqual(f2.dat.hex(), FACTORY_RSA2[limit is not None] + f"{0x80 | syncid:02x}")

  def test_frames_decode(self):
    cp = CANParser(PT_DBC, [("RSA1", float('nan')), ("RSA2", float('nan'))], 0)
    for i, limit in enumerate((25, 55, 85, None)):
      cp.update([(i + 1, [(f.address, f.dat, f.src) for f in rsa_frames(limit, 9)])])
      rsa1, rsa2 = cp.vl["RSA1"], cp.vl["RSA2"]
      self.assertEqual((rsa1["TSGN1"], rsa1["SPDVAL1"], rsa1["TSGNHLT1"]), (36 if limit else 0, limit or 0, 0))
      self.assertEqual((rsa2["SGNNUMP"], rsa2["SPDUNT"]), (1 if limit else 0, 2))
      self.assertEqual((rsa1["SYNCID1"], rsa2["SYNCID2"]), (9, 9))
      for s in ("TSGNGRY1", "SPLSGN1", "SPLSGN2", "TSGN2", "TSGNGRY2", "TSGNHLT2", "SPDVAL2", "BZRRQ_P", "BZRRQ_A"):
        self.assertEqual(rsa1[s], 0, s)
      for s in ("TSGN3", "TSGNGRY3", "TSGNHLT3", "SPLSGN3", "SPLSGN4", "TSGN4", "TSGNGRY4", "TSGNHLT4", "DPSGNREQ", "SGNNUMA",
                "TSRWMSG"):
        self.assertEqual(rsa2[s], 0, s)


class TestRsaCarState(unittest.TestCase):
  def test_accept_after_repeat(self):
    f = NavFeed()
    self.assertEqual(set(f.frame(0x47, 1, 1)), {None})    # the first frame is only a candidate
    self.assertEqual(set(f.frame(0x47, 1, 1)), {30})      # repeated: on the sign from that frame on
    self.assertEqual(set(f.frame(0x48, 1, 2)), {30})      # head-unit update: candidate
    self.assertEqual(set(f.frame(0x48, 1, 2)), {35})

  def test_no_limit_blanks_after_hold(self):
    f = NavFeed()
    f.frame(0x48, 1, 1)
    f.frame(0x48, 1, 1)
    signs = f.frame(0, 0, 2) + f.frame(0, 0, 2)            # "no limit" accepted at the second frame ...
    while len(signs) < NAV_PERIOD_FRAMES + NAV_NONE_HOLD_FRAMES + 10:
      signs += f.frame(0, 0, 2)
    self.assertIsNone(f.cs.nav_speed_limit)
    # ... and the sign stays for NAV_NONE_HOLD_FRAMES cycles after that, counting the accepting cycle
    self.assertEqual(signs.index(None), NAV_PERIOD_FRAMES + NAV_NONE_HOLD_FRAMES - 1)
    self.assertEqual(set(signs[signs.index(None):]), {None})

  def test_short_map_gap_keeps_sign(self):
    # DRCC Mid Speed Down Hill: 30 mph, then four no-limit frames (2 s), then 30 mph again
    f = NavFeed()
    f.frame(0x47, 1, 1)
    signs = f.frame(0x47, 1, 1)
    for _ in range(4):
      signs += f.frame(0, 0, 2)
    for _ in range(4):
      signs += f.frame(0x47, 1, 3)
    self.assertEqual(set(signs), {30})

  def test_new_limit_during_hold(self):
    f = NavFeed()
    f.frame(0x47, 1, 1)
    f.frame(0x47, 1, 1)
    f.frame(0, 0, 2)
    self.assertEqual(set(f.frame(0, 0, 2)), {30})          # no limit accepted, sign held
    self.assertEqual(set(f.frame(0x4A, 1, 3)), {30})       # candidate
    self.assertEqual(set(f.frame(0x4A, 1, 3)), {45})       # accepted: replaces the held sign at once
    for _ in range(2 * NAV_NONE_HOLD_FRAMES // NAV_PERIOD_FRAMES):
      self.assertEqual(set(f.frame(0x4A, 1, 3)), {45})     # the old hold does not blank it later

  def test_odd_frames_never_shown(self):
    f = NavFeed()
    f.frame(0x47, 1, 4)
    f.frame(0x47, 1, 4)
    self.assertEqual(set(f.frame(0x4E, 1, 4)), {30})       # changed value, same update counter: one frame
    self.assertEqual(set(f.frame(0x4E, 1, 4)), {30})       # ... and a second: still not taken
    for _ in range(3):
      self.assertEqual(set(f.frame(0x47, 1, 4)), {30})     # back again: 65 was never shown
    self.assertEqual(set(f.frame(0x4E, 1, 5)), {30})       # a single frame with a new counter is not enough either
    for _ in range(2):
      self.assertEqual(set(f.frame(0x47, 1, 6)), {30})

  def test_same_counter_change_taken_on_third_frame(self):
    # a head unit that changes the value without advancing the counter must not leave a stale sign up
    f = NavFeed()
    f.frame(0x47, 1, 4)
    f.frame(0x47, 1, 4)
    self.assertEqual(set(f.frame(0x4E, 1, 4)), {30})
    self.assertEqual(set(f.frame(0x4E, 1, 4)), {30})
    self.assertEqual(set(f.frame(0x4E, 1, 4)), {65})       # third identical frame: taken
    f.frame(0, 0, 4)
    f.frame(0, 0, 4)
    signs = f.frame(0, 0, 4)                               # "no limit" the same way: accepted here, then held 3 s
    for _ in range(NAV_NONE_HOLD_FRAMES // NAV_PERIOD_FRAMES + 1):
      signs += f.frame(0, 0, 4)
    self.assertEqual(signs[0], 65)
    self.assertEqual(signs.index(None), NAV_NONE_HOLD_FRAMES - 1)

  def test_unknown_unit_and_bounds(self):
    for code, flag in ((0x47, 2), (0x53, 1)):              # another byte-1 value; 90 mph (outside 5-85)
      f = NavFeed()
      f.frame(0x47, 1, 1)
      f.frame(0x47, 1, 1)
      signs = []
      for _ in range(2 + NAV_NONE_HOLD_FRAMES // NAV_PERIOD_FRAMES + 1):
        signs += f.frame(code, flag, 2)
      self.assertIsNone(f.cs.nav_speed_limit, (code, flag))
      self.assertEqual(signs.index(None), NAV_PERIOD_FRAMES + NAV_NONE_HOLD_FRAMES - 1, (code, flag))   # treated as no limit

  def test_timeout(self):
    f = NavFeed()
    f.frame(0x46, 1, 1)
    f.step(nav_frame(0x46, 1, 1))
    for _ in range(NAV_TIMEOUT_FRAMES):
      self.assertEqual(f.step(), 25)
    self.assertIsNone(f.step())                            # 2 s without a frame: blank at once
    self.assertEqual(set(f.frame(0x46, 1, 1)), {None})     # needs the repeat again
    self.assertEqual(set(f.frame(0x46, 1, 1)), {25})

  def test_timeout_cuts_hold_short(self):
    f = NavFeed()
    f.frame(0x46, 1, 1)
    f.frame(0x46, 1, 1)
    f.frame(0, 0, 2)
    f.step(nav_frame(0, 0, 2))                             # no limit accepted, then the head unit goes silent
    signs = [f.step() for _ in range(NAV_NONE_HOLD_FRAMES)]
    self.assertLess(NAV_TIMEOUT_FRAMES, NAV_NONE_HOLD_FRAMES)
    self.assertEqual(signs.index(None), NAV_TIMEOUT_FRAMES)

  def test_no_frame_yet(self):
    f = NavFeed()
    for _ in range(3 * NAV_TIMEOUT_FRAMES):
      self.assertIsNone(f.step())

  def test_disabled_on_other_cars(self):
    cs = RsaCarState(*car_params(flags=0))
    cs.update_rsa({})                                      # does not touch the parsers at all
    self.assertIsNone(cs.rsa_limit)


class TestRsaCarController(unittest.TestCase):
  def test_gating(self):
    for args in ({"flags": 0}, {"op_long": False}, {"interceptor": True},
                 {"flags": ToyotaFlags.UNSUPPORTED_DSU | ToyotaFlags.SECOC}):
      cc = RsaCarController(*car_params(**args))
      self.assertEqual(cc.create_rsa_msgs(cs_with(55), 0), [], args)
    self.assertEqual(len(RsaCarController(*car_params()).create_rsa_msgs(cs_with(55), 0)), 2)

  def test_schedule_and_syncid(self):
    cc = RsaCarController(*car_params())
    sent = {}
    for frame in range(1600):
      limit = 35 if frame < 437 else None if frame < 911 else 45
      msgs = cc.create_rsa_msgs(cs_with(limit), frame)
      if msgs:
        sent[frame] = msgs
    self.assertEqual(sorted(sent), sorted(set(range(0, 1600, 100)) | {437, 911}))   # 1 Hz, and at once on a change
    self.assertTrue(all([m.address for m in msgs] == [RSA1_ADDR, RSA2_ADDR] for msgs in sent.values()))   # always both
    syncids = [sent[f][0].dat[7] for f in sorted(sent)]
    self.assertEqual(syncids, [(i % 15) + 1 for i in range(len(syncids))])   # 1-15, wrapping, advanced on every send
    self.assertTrue(all(a != b for a, b in zip(syncids, syncids[1:], strict=False)))
    self.assertTrue(all(m[1].dat[7] == 0x80 | m[0].dat[7] for m in sent.values()))
    self.assertEqual((sent[400][0].dat[2], sent[437][0].dat[:7], sent[911][0].dat[2]), (35, bytes(7), 45))

  def test_no_highlight(self):
    cc = RsaCarController(*car_params())
    for i, limit in enumerate((25, 55, 85)):
      self.assertEqual(cc.create_rsa_msgs(cs_with(limit), i * RSA_PERIOD_FRAMES)[0].dat[1], 0)

  def test_first_send_at_once(self):
    cc = RsaCarController(*car_params())
    self.assertEqual(len(cc.create_rsa_msgs(cs_with(None), 37)), 2)
    self.assertEqual(cc.create_rsa_msgs(cs_with(None), 38), [])


class TestRsaInterface(unittest.TestCase):
  """RSA through the real Lexus IS CarInterface: parser registration, CarState and CarController hooks"""
  def interface(self, smart_dsu=True, alpha_long=True, eps_len=None):
    fingerprint = {0: {0x2FF: 8} if smart_dsu else {}, 1: {}, 2: {}}
    if eps_len is not None:
      fingerprint[0][0x262] = eps_len
    CarInterface = interfaces[CAR.LEXUS_IS]
    CP = CarInterface.get_params(CAR.LEXUS_IS, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.LEXUS_IS, fingerprint, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
    return CarInterface(CP, CP_SP)

  def run_frames(self, ci, frames):
    CC = structs.CarControl().as_reader()
    CC_SP = structs.CarControlSP()
    sends = []
    for i in range(frames):
      nanos = round((i + 1) * DT_CTRL * 1e9)
      cans = [CanData(NAV_ADDR, nav_frame(0x4C, 1, 7), 0)] if i % NAV_PERIOD_FRAMES == 0 else []
      ci.update([(nanos, cans)])
      sends.append([CanData(*m) for m in ci.apply(CC, CC_SP, nanos)[1]])   # the packer returns plain tuples
    return sends

  def test_op_long(self):
    for eps_len in (None, 5, 8):
      ci = self.interface(eps_len=eps_len)
      self.assertTrue(ci.CP.openpilotLongitudinalControl)
      self.assertEqual(bool(ci.CP_SP.flags & ToyotaFlagsSP.TSS2_EPS), eps_len == 8)
      self.assertTrue(ci.can_parsers[Bus.pt].message_states[NAV_ADDR].ignore_alive)   # absent head unit: no CAN error
      sends = self.run_frames(ci, 120)
      self.assertEqual(ci.CS.rsa_limit, 55)
      rsa = [(i, m) for i, s in enumerate(sends) for m in s if m.address in (RSA1_ADDR, RSA2_ADDR)]
      self.assertEqual([(i, m.address) for i, m in rsa], [(0, RSA1_ADDR), (0, RSA2_ADDR), (50, RSA1_ADDR), (50, RSA2_ADDR),
                                                          (100, RSA1_ADDR), (100, RSA2_ADDR)])
      self.assertEqual(rsa[0][1].dat[:7], bytes(7))                  # nothing accepted yet
      self.assertEqual(rsa[2][1].dat[:3], bytes([0x24, 0, 55]))      # accepted at the second frame: sent at once
      self.assertEqual([m.dat[7] for _, m in rsa], [1, 0x81, 2, 0x82, 3, 0x83])
      self.assertTrue(all(m.src == 0 for i, m in rsa))

  def test_stock_long(self):
    for smart_dsu, alpha_long in ((False, True), (True, False)):
      ci = self.interface(smart_dsu, alpha_long)
      self.assertFalse(ci.CP.openpilotLongitudinalControl)
      sends = self.run_frames(ci, 120)
      self.assertEqual(ci.CS.rsa_limit, 55)                          # parsed, not sent: the camera owns the display
      self.assertFalse(any(m.address in (RSA1_ADDR, RSA2_ADDR) for s in sends for m in s))

  def test_other_toyota(self):
    CarInterface = interfaces[CAR.TOYOTA_RAV4]
    fingerprint = {0: {}, 1: {}, 2: {}}
    CP = CarInterface.get_params(CAR.TOYOTA_RAV4, fingerprint, [], alpha_long=True, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TOYOTA_RAV4, fingerprint, [], alpha_long=True, is_release_sp=False, docs=False)
    ci = CarInterface(CP, CP_SP)
    self.assertNotIn(NAV_ADDR, ci.can_parsers[Bus.pt].addresses)
    sends = self.run_frames(ci, 120)
    self.assertFalse(any(m.address in (RSA1_ADDR, RSA2_ADDR) for s in sends for m in s))


if __name__ == "__main__":
  unittest.main()
