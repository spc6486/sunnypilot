"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


class ToyotaSafetyFlagsSP:
  DEFAULT = 0
  UNSUPPORTED_DSU = 1
  SMART_DSU_IS = 2  # SmartDSU-IS hardware detected - enables 0x2FE TX


# SmartDSU-IS CAN message IDs
SDSU_PRESENCE_MSG = 0x2FF
SDSU_ACCEL_CMD = 0x2FE
LEXUS_IS_MASS = 1700
