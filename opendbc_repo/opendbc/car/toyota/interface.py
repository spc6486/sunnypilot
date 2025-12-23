from opendbc.car import Bus, structs, get_safety_config, uds
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, TSS2_CAR, RADAR_ACC_CAR, NO_DSU_CAR, \
                                                  MIN_ACC_SPEED, EPS_SCALE, UNSUPPORTED_DSU_CAR, NO_STOP_TIMER_CAR, ANGLE_CONTROL_CAR, \
                                                  ToyotaSafetyFlags
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.sunnypilot.car.toyota.values import ToyotaSafetyFlagsSP, SDSU_PRESENCE_MSG

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "toyota"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = EPS_SCALE[candidate]

    # BRAKE_MODULE is on a different address for these cars
    if DBC[candidate][Bus.pt] == "toyota_new_mc_pt_generated":
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.ALT_BRAKE.value

    if ret.flags & ToyotaFlags.SECOC.value:
      ret.secOcRequired = True
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.SECOC.value
      ret.dashcamOnly = is_release

    if candidate in ANGLE_CONTROL_CAR:
      ret.steerControlType = SteerControlType.angle
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.LTA.value

      # LTA control can be more delayed and winds up more often
      ret.steerActuatorDelay = 0.18
      ret.steerLimitTimer = 0.8
    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
      ret.steerLimitTimer = 0.4

    stop_and_go = candidate in TSS2_CAR

    # In TSS2 cars, the camera does long control
    found_ecus = [fw.ecu for fw in car_fw]
    ret.enableDsu = len(found_ecus) > 0 and Ecu.dsu not in found_ecus and candidate not in (NO_DSU_CAR | UNSUPPORTED_DSU_CAR)

    if Ecu.hybrid in found_ecus:
      ret.flags |= ToyotaFlags.HYBRID.value

    if candidate in (TSS2_CAR - RADAR_ACC_CAR):
      ret.flags |= ToyotaFlags.SMART_DSU.value

    # SmartDSU-IS detection: Check for 0x2FF presence message in fingerprint
    # This indicates SmartDSU-IS hardware is present and broadcasting
    smart_dsu_is_detected = SDSU_PRESENCE_MSG in fingerprint[0]

    # Enable stop-and-go for UNSUPPORTED_DSU cars with SmartDSU-IS hardware
    if smart_dsu_is_detected and candidate in UNSUPPORTED_DSU_CAR:
      # SmartDSU-IS enables full longitudinal control including stop-and-go
      ret.openpilotLongitudinalControl = True
      ret.pcmCruise = False
      stop_and_go = True

      # CRITICAL: Keep STOCK_LONGITUDINAL set to avoid relay check on 0x343
      # SmartDSU-IS uses 0x2FE instead, which has check_relay=false
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    # TODO: these models can do stop and go, but unclear if it requires sDSU or unplugging DSU.
    #  For now, don't list stop and go functionality in the docs
    if ret.flags & ToyotaFlags.SNG_WITHOUT_DSU:
      stop_and_go = stop_and_go or (ret.enableDsu and not docs)

    ret.centerToFront = ret.wheelbase * 0.44

    # TODO: Some TSS-P platforms have BSM, but are flipped based on region or driving direction.
    # Detect flipped signals and enable for C-HR and others
    ret.enableBsm = 0x3F6 in fingerprint[0] and candidate in TSS2_CAR

    # No radar dbc for cars without DSU which are not TSS 2.0
    # TODO: make an adas dbc file for dsu-less models
    ret.radarUnavailable = Bus.radar not in DBC[candidate] or candidate in (NO_DSU_CAR - TSS2_CAR)

    # since we don't yet parse radar on TSS2/TSS-P radar-based ACC cars, gate longitudinal behind experimental toggle
    if candidate in (RADAR_ACC_CAR | NO_DSU_CAR):
      ret.alphaLongitudinalAvailable = candidate in RADAR_ACC_CAR

      # Disabling radar is only supported on TSS2 radar-ACC cars
      if alpha_long and candidate in RADAR_ACC_CAR:
        ret.flags |= ToyotaFlags.DISABLE_RADAR.value

    # openpilot longitudinal enabled by default:
    #  - cars w/ DSU disconnected
    #  - TSS2 cars with camera sending ACC_CONTROL where we can block it
    #  - UNSUPPORTED_DSU cars with SmartDSU-IS hardware (already set above)
    # openpilot longitudinal behind experimental long toggle:
    #  - TSS2 radar ACC cars (disables radar)

    if not smart_dsu_is_detected:  # Don't override if SmartDSU-IS already enabled it
      ret.openpilotLongitudinalControl = ret.enableDsu or \
        candidate in (TSS2_CAR - RADAR_ACC_CAR) or \
        bool(ret.flags & ToyotaFlags.DISABLE_RADAR.value)

    # Auto resume from stop: enabled for NO_STOP_TIMER_CAR or SmartDSU-IS cars
    # SmartDSU-IS bypasses DSU's low-speed lockout, PCM honors FORCE at all speeds
    ret.autoResumeSng = ret.openpilotLongitudinalControl and (candidate in NO_STOP_TIMER_CAR or smart_dsu_is_detected)

    if not ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if stop_and_go else MIN_ACC_SPEED

    # Stopping/starting parameters for smooth stop-and-go behavior
    if candidate in TSS2_CAR or smart_dsu_is_detected:
      if candidate in TSS2_CAR:
        ret.flags |= ToyotaFlags.RAISED_ACCEL_LIMIT.value

      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25
      ret.stoppingDecelRate = 0.3  # reach stopping target smoothly

      # Hybrids have much quicker longitudinal actuator response
      if ret.flags & ToyotaFlags.HYBRID.value:
        ret.longitudinalActuatorDelay = 0.05

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, docs: bool) -> structs.CarParamsSP:
    # Set UNSUPPORTED_DSU flag for cars that use 0x283 instead of 0x343
    if candidate in UNSUPPORTED_DSU_CAR:
      ret.safetyParam |= ToyotaSafetyFlagsSP.UNSUPPORTED_DSU

    # SmartDSU-IS detection: Check for 0x2FF presence message
    # When detected, set the SMART_DSU_IS flag to enable 0x2FE TX in panda safety
    smart_dsu_is_detected = SDSU_PRESENCE_MSG in fingerprint[0]
    if smart_dsu_is_detected and candidate in UNSUPPORTED_DSU_CAR:
      ret.safetyParam |= ToyotaSafetyFlagsSP.SMART_DSU_IS

    if candidate in (CAR.TOYOTA_WILDLANDER, ):
      stock_cp.lateralTuning.init('pid')
      stock_cp.lateralTuning.pid.kiBP = [0.0]
      stock_cp.lateralTuning.pid.kpBP = [0.0]
      stock_cp.lateralTuning.pid.kpV = [0.6]
      stock_cp.lateralTuning.pid.kiV = [0.1]
      stock_cp.lateralTuning.pid.kf = 0.00007818594

    return ret

  @staticmethod
  def init(CP, CP_SP, can_recv, can_send, communication_control=None):
    # disable radar if alpha longitudinal toggled on radar-ACC car
    if CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if communication_control is None:
        communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
      disable_ecu(can_recv, can_send, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    # re-enable radar if alpha longitudinal toggled on radar-ACC car
    communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_ENABLE_TX, uds.MESSAGE_TYPE.NORMAL])
    CarInterface.init(CP, can_recv, can_send, communication_control)
