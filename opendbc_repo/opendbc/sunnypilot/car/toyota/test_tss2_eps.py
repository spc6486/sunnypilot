"""
TSS2 power-steering ECU on a TSS-P Toyota (ToyotaFlagsSP.TSS2_EPS): detection from the EPS_STATUS (0x262) frame length
seen at fingerprinting, the DBC variant carstate parses with, and that both EPS versions parse without CAN errors.
"""
import unittest

from opendbc.can.dbc import DBC as DBCFile
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.toyotacan import toyota_checksum
from opendbc.car.toyota.values import CAR, DBC
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, TSS2_EPS_DBC

EPS_STATUS = 0x262


def eps_status(length: int, lka_state: int = 5) -> bytes:
  """EPS_STATUS as the car sends it: LKA_STATE in byte 3 bits 7-1, TYPE in bit 0, Toyota checksum in the last byte"""
  dat = bytearray(length)
  dat[3] = (lka_state << 1) | 1
  dat[-1] = toyota_checksum(EPS_STATUS, None, dat)
  return bytes(dat)


def params(candidate, eps_length: int | None, smart_dsu: bool = False):
  fingerprint = gen_empty_fingerprint()
  if eps_length is not None:
    fingerprint[0][EPS_STATUS] = eps_length
  if smart_dsu:
    fingerprint[0][0x2FF] = 8
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=smart_dsu, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], alpha_long=smart_dsu, is_release_sp=False, docs=False)
  return CP, CP_SP


class TestTss2Eps(unittest.TestCase):
  def test_flag_values_unique(self):
    # an IntFlag member with an existing member's value would silently become an alias of it
    self.assertEqual(len({f.value for f in ToyotaFlagsSP}), len(ToyotaFlagsSP.__members__))

  def test_variant_matches_stock_except_eps_status(self):
    # the variant must track its stock DBC: same messages and signals, except EPS_STATUS's length and checksum position
    for stock_name, variant_name in TSS2_EPS_DBC.items():
      stock, variant = DBCFile(stock_name), DBCFile(variant_name)
      with self.subTest(dbc=stock_name):
        self.assertEqual(stock.name_to_msg.keys(), variant.name_to_msg.keys())
        for name, msg in stock.name_to_msg.items():
          other = variant.name_to_msg[name]
          if name != "EPS_STATUS":
            self.assertEqual(msg, other, name)
            continue
          self.assertEqual((msg.address, msg.size, other.size), (EPS_STATUS, 5, 8))
          self.assertEqual(msg.sigs.keys(), other.sigs.keys())
          for sig_name, sig in msg.sigs.items():
            if sig_name == "CHECKSUM":
              self.assertEqual((sig.start_bit, other.sigs[sig_name].start_bit), (39, 63))  # last byte of each
              self.assertEqual((sig.size, sig.type), (other.sigs[sig_name].size, other.sigs[sig_name].type))
            else:
              self.assertEqual(sig, other.sigs[sig_name], sig_name)
        self.assertEqual(stock.vals, variant.vals)

  def test_detection(self):
    for candidate in CAR:
      eligible = DBC[candidate][Bus.pt] in TSS2_EPS_DBC
      for eps_length in (None, 5, 8):
        for smart_dsu in (False, True):
          with self.subTest(candidate=candidate.value, eps_length=eps_length, smart_dsu=smart_dsu):
            CP, CP_SP = params(candidate, eps_length, smart_dsu)
            expected = eligible and eps_length == 8
            self.assertEqual(bool(CP_SP.flags & ToyotaFlagsSP.TSS2_EPS), expected)
            self.assertEqual(bool(CP_SP.flags & ToyotaFlagsSP.SMART_DSU), smart_dsu)

    # the retrofit car as it runs: C-SDSU and openpilot longitudinal on, TSS2 EPS
    CP, CP_SP = params(CAR.LEXUS_IS, 8, smart_dsu=True)
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertEqual(CP_SP.flags & (ToyotaFlagsSP.SMART_DSU | ToyotaFlagsSP.TSS2_EPS), ToyotaFlagsSP.SMART_DSU | ToyotaFlagsSP.TSS2_EPS)

  def test_both_eps_versions_parse(self):
    # a car parses only the EPS_STATUS length it was detected with; the other length fails its checksum
    for eps_length in (5, 8):
      with self.subTest(eps_length=eps_length):
        CP, CP_SP = params(CAR.LEXUS_IS, eps_length)
        CI = CarInterface(CP, CP_SP)
        cp = CI.can_parsers[Bus.pt]
        self.assertEqual(cp.dbc_name, "toyota_tnga_k_tss2_eps_pt_generated" if eps_length == 8 else "toyota_tnga_k_pt_generated")

        other_length = {5: 8, 8: 5}[eps_length]
        for sent_length, accepted in ((eps_length, True), (other_length, False)):
          CI = CarInterface(CP, CP_SP)
          cp = CI.can_parsers[Bus.pt]
          self.assertEqual(cp.vl["EPS_STATUS"]["LKA_STATE"], 0)  # registers the message
          for i in range(1, 26):  # 1 s at 25 Hz
            cp.update([(i * 40_000_000, [(EPS_STATUS, eps_status(sent_length), 0)])])
          self.assertEqual(cp.vl["EPS_STATUS"]["LKA_STATE"], 5 if accepted else 0)
          self.assertEqual(len(cp.message_states[EPS_STATUS].timestamps) > 0, accepted)


if __name__ == "__main__":
  unittest.main()
