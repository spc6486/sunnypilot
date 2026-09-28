"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class ToyotaFlagsSP(IntFlag):
  SMART_DSU = 1
  RADAR_CAN_FILTER = 2
  ZSS = 4
  STOCK_LONGITUDINAL = 8
  STOP_AND_GO_HACK = 16
  TSS2_EPS = 32
  ENHANCED_BSM = 64


# DBCs that define the stock 5-byte EPS_STATUS, mapped to the variant with the 8-byte EPS_STATUS a TSS2 power-steering
# ECU sends (checksum in the last byte). carstate parses with the variant when ToyotaFlagsSP.TSS2_EPS is detected.
TSS2_EPS_DBC = {
  "toyota_tnga_k_pt_generated": "toyota_tnga_k_tss2_eps_pt_generated",
}


class ToyotaSafetyFlagsSP:
  DEFAULT = 0
  UNSUPPORTED_DSU = 1
  GAS_INTERCEPTOR = 2
  ENHANCED_BSM = 4
