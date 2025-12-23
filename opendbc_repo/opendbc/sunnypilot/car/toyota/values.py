"""
Toyota sunnypilot-specific values and constants.

This file contains sunnypilot-specific safety flags and constants for Toyota vehicles.
"""

from enum import IntFlag


class ToyotaSafetyFlagsSP(IntFlag):
  """
  Sunnypilot-specific safety flags for Toyota vehicles.

  These flags are passed via safetyParam to the panda safety code
  through current_safety_param_sp.

  IMPORTANT: These values must match the constants in:
    opendbc_repo/opendbc/safety/modes/toyota.h

  In toyota.h:
    const int TOYOTA_PARAM_SP_UNSUPPORTED_DSU = 1;
    const int TOYOTA_PARAM_SP_SMART_DSU_IS = 2;
  """
  # Bit 0: UNSUPPORTED_DSU car (Lexus IS, RC, GS F)
  # These cars use PRE_COLLISION (0x283) for longitudinal instead of ACC_CONTROL (0x343)
  UNSUPPORTED_DSU = 1

  # Bit 1: SmartDSU-IS hardware detected
  # When set, allows TX of 0x2FE (SDSU_ACCEL_CMD) for ESP32 communication
  # Used to enable stop-and-go on UNSUPPORTED_DSU cars
  SMART_DSU_IS = 2


# SmartDSU-IS CAN message IDs
SDSU_PRESENCE_MSG = 0x2FF   # ESP32 broadcasts this at 10Hz to indicate presence
SDSU_ACCEL_CMD = 0x2FE      # OpenPilot sends accel commands to ESP32 on this address

# Vehicle mass for force calculations (Lexus IS 2014-2020)
LEXUS_IS_MASS = 1700  # kg (approximate curb weight)
