import math
import numpy as np

from opendbc.can import CANPacker, CanData
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits, structs, make_tester_present_msg
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota.values import CAR, STATIC_DSU_MSGS, ToyotaFlags, CarControllerParams, NO_STOP_TIMER_CAR, \
                                                  UNSUPPORTED_DSU_CAR
from opendbc.car.toyota import toyotacan
from opendbc.sunnypilot.car.toyota.secoc_long import SecOCLongCarController
from opendbc.sunnypilot.car.toyota.values import ToyotaSafetyFlagsSP

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# constants for pitch compensation
ACCELERATION_DUE_TO_GRAVITY = 9.81  # m/s^2
# always 0 for now, but this can be used in the future
RATE_LIMIT_ACTIVE = 0.  # m/s^2/s

# Max user torque before blocking lane change (safety concern since it could confuse ACC into thinking you're steering)
MAX_USER_TORQUE = 500
ACCEL_WINDUP_LIMIT = 0.3  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = 0.045  # m/s^2 / frame
ACCEL_PID_UNWIND = 0.02  # m/s^2 / frame


def rate_limit(new_value, last_value, dw_step, up_step):
  return float(np.clip(new_value, last_value - dw_step, last_value + up_step))


def get_long_tune(CP, params):
  # Tuning parameters for the longitudinal PID controller
  if CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT.value:
    kiBP = [0., 5., 20., 30.]
    kiV = [0.5, 0.25, 0.05, 0.02]
  elif CP.flags & ToyotaFlags.HYBRID.value:
    kiBP = [0., 5., 35.]
    kiV = [0.5, 0.25, 0.05]
  else:
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]

  return PIDController(0.0, (kiBP, kiV), k_f=1.0,
                       pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
                       rate=1 / (DT_CTRL * 3))


class CarController(CarControllerBase, SecOCLongCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    SecOCLongCarController.__init__(self, CP)
    self.params = CarControllerParams(self.CP)
    self.last_torque = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.distance_button = 0

    # *** start long control state ***
    self.long_pid = get_long_tune(self.CP, self.params)
    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 3)
    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)

    self.accel = 0
    self.prev_accel = 0
    # *** end long control state ***

    self.packer = CANPacker(dbc_names[Bus.pt])

    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_prev_reset_counter = 0

    # SmartDSU-IS: Counter for 0x2FE messages (0-15)
    self.sdsu_counter = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])

    # *** control msgs ***
    can_sends = []

    SecOCLongCarController.update(self, CS, can_sends, self.secoc_prev_reset_counter)

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

    # *** steer torque ***
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.last_torque, CS.out.steeringTorque, self.params)

    # >100 degree/s steering fault prevention
    self.steer_rate_counter, apply_torque_limited = \
      self.common_steer_torque_rate_limiter(apply_torque, self.last_torque, CS.out.steeringAngleDeg, CS.out.steeringPressed, self.steer_rate_counter)

    if not lat_active:
      apply_torque = 0
      apply_torque_limited = 0

    # Toyota LKA torque limiter
    apply_torque = self.common_toyota_torque_limiter(apply_torque, CS.out.steeringAngleDeg)
    self.last_torque = apply_torque_limited

    if self.frame % 2 == 0 and self.CP.carFingerprint not in (ANGLE_CONTROL_CAR if hasattr(self, 'ANGLE_CONTROL_CAR') else set()):
      # steer cmd
      can_sends.append(toyotacan.create_steer_command(self.packer, apply_torque, lat_active))

    # *** LTA steering ***
    if self.CP.steerControlType == structs.CarParams.SteerControlType.angle:
      if CC.latActive:
        self.last_angle = actuators.steeringAngleDeg
      lta_active = CC.latActive and abs(CS.out.steeringTorque) < self.params.MAX_LTA_DRIVER_TORQUE_ALLOWANCE

      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < self.params.MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

    # *** gas and brake ***

    # Check if SmartDSU-IS is active (moved earlier for standstill logic)
    smart_dsu_is_active = bool(self.CP_SP.safetyParam & ToyotaSafetyFlagsSP.SMART_DSU_IS)

    # on entering standstill, send standstill request
    # Skip for NO_STOP_TIMER_CAR and SmartDSU-IS cars - they auto-resume from planner commands
    if CS.out.standstill and not self.last_standstill and \
       (self.CP.carFingerprint not in NO_STOP_TIMER_CAR) and not smart_dsu_is_active:
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_standstill = CS.out.standstill

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 3 == 0:
        # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
        if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
          desired_distance = 4 - hud_control.leadDistanceBars
          if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
            self.distance_button = not self.distance_button
          else:
            self.distance_button = 0

        # internal PCM gas command can get stuck unwinding from negative accel so we apply a generous rate limit
        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
        self.prev_accel = pcm_accel_cmd

        # calculate amount of acceleration PCM should apply to reach target, given pitch.
        # clipped to only include downhill angles, avoids erroneously unsetting PERMIT_BRAKING when stopping on uphills
        accel_due_to_pitch = math.sin(min(self.pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY
        # TODO: on uphills this sometimes sets PERMIT_BRAKING low not considering the creep force
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch

        # GVC does not overshoot ego acceleration when starting from stop, but still has a similar delay
        if not self.CP.flags & ToyotaFlags.SECOC.value:
          a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo]))
        else:
          a_ego_blended = CS.out.aEgo

        # wind down integral when approaching target for step changes and smooth ramps to reduce overshoot
        prev_aego = self.aego.x
        self.aego.update(a_ego_blended)
        j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)

        future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
        a_ego_future = a_ego_blended + j_ego * future_t

        if CC.longActive:
          # constantly slowly unwind integral to recover from large temporary errors
          self.long_pid.i -= ACCEL_PID_UNWIND * float(np.sign(self.long_pid.i))

          error_future = pcm_accel_cmd - a_ego_future
          pcm_accel_cmd = self.long_pid.update(error_future,
                                               speed=CS.out.vEgo,
                                               feedforward=pcm_accel_cmd,
                                               freeze_integrator=actuators.longControlState != LongCtrlState.pid)
        else:
          self.long_pid.reset()

        # Along with rate limiting positive jerk above, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min < 0.2 or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min > 0.3:
          self.permit_braking = False

        pcm_accel_cmd = float(np.clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

        # SmartDSU-IS: Send 0x2FE instead of standard ACC_CONTROL
        if smart_dsu_is_active:
          # Send SmartDSU-IS accel command (0x2FE)
          can_sends.append(toyotacan.create_sdsu_accel_command(
            accel=pcm_accel_cmd,
            permit_braking=self.permit_braking,
            release_standstill=not self.standstill_req,
            cancel_req=pcm_cancel_cmd,
            long_active=CC.longActive,
            counter=self.sdsu_counter
          ))
          self.sdsu_counter = (self.sdsu_counter + 1) % 16
        else:
          # Standard ACC_CONTROL (0x343)
          can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.permit_braking, self.standstill_req, lead,
                                                          CS.acc_type, fcw_alert, self.distance_button, self.SECOC_LONG))
        self.accel = pcm_accel_cmd

    else:
      # we can spam can to cancel the system even if we are using lat only control
      if pcm_cancel_cmd:
        if smart_dsu_is_active:
          # SmartDSU-IS: Send cancel via 0x2FE
          can_sends.append(toyotacan.create_sdsu_accel_command(
            accel=0,
            permit_braking=True,
            release_standstill=True,
            cancel_req=True,
            long_active=False,
            counter=self.sdsu_counter
          ))
          self.sdsu_counter = (self.sdsu_counter + 1) % 16
        elif self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button,
                                                          self.SECOC_LONG))

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, CC.latActive, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    if self.CP.enableDsu:
      for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
        if self.frame % fr_step == 0 and self.CP.carFingerprint in cars:
          can_sends.append(CanData(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
