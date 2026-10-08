#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.lexus_is import features as lexus_is_features

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -1.2
# Lexus IS branch, shaped set-speed law (setting set_speed_law, sunnypilot/lexus_is/features.py): the acceleration toward
# the set speed tapers with the speed error and aims just below it; a small overspeed is shed by coasting before any
# braking request; rises are jerk-limited; a slow trim removes the steady error a grade leaves.
J_CRUISE_VALS_SHAPED = [1.2, 1.0, 0.6, 0.5]  # m/s^3, at A_CRUISE_MAX_BP
J_CRUISE_UNWIND_FACTOR = 2.0          # a positive cruise acceleration may decrease this much faster
J_CRUISE_RELEASE = 3.0                # m/s^3: a negative cruise acceleration (started from a braking output) releases at this rate
CRUISE_GAIN_BP = [5., 20.]            # m/s
CRUISE_GAIN_V = [0.6, 0.3]            # m/s^2 per m/s of speed error below the set speed
CRUISE_APPROACH_MARGIN = 0.1          # m/s: the approach aims this far below the set speed
CRUISE_COAST_BAND = 0.42              # m/s (1.5 km/h) above the set speed: coasting only
CRUISE_COAST_ACCEL = -0.05            # m/s^2: request at the top of the coast band
CRUISE_OVER_GAIN = 0.3                # m/s^2 per m/s of overspeed beyond the coast band
CRUISE_KI = 0.06                      # m/s^2 per (m/s * s): slow trim of the steady error (grades)
CRUISE_I_MAX = 0.3                    # m/s^2: trim bound
CRUISE_I_ZONE = 1.5                   # m/s: the trim integrates only within this speed error of the set speed
CRUISE_I_STEADY_A = 0.1               # m/s^2: ... and only while the car holds its speed (|aEgo| below this); it is held, not
                                      # decayed, while the car accelerates or slows, so it cannot wind up on an approach
CRUISE_I_LEAK_TC = 3.0                # s: otherwise the trim decays with this time constant
CRUISE_TRIM_LOG_EVERY = 20            # planner cycles (1 s at 20 Hz): the trim is logged to cloudlog at this interval
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def shaped_cruise_target(v_err, v_ego, max_accel):
  if v_err > CRUISE_APPROACH_MARGIN:
    k = float(np.interp(v_ego, CRUISE_GAIN_BP, CRUISE_GAIN_V))
    return min(k * (v_err - CRUISE_APPROACH_MARGIN), max_accel)
  if v_err > 0.:
    return min(0., max_accel)
  if v_err > -CRUISE_COAST_BAND:
    return min(CRUISE_COAST_ACCEL * (-v_err / CRUISE_COAST_BAND), max_accel)
  return min(max(CRUISE_COAST_ACCEL + CRUISE_OVER_GAIN * (v_err + CRUISE_COAST_BAND), A_CRUISE_MIN), max_accel)


def shaped_cruise_accel(v_err, v_ego, max_accel, a_cruise_prev, dt, cruise_trim):
  if cruise_trim > 0. and v_err < 0.:
    # above the set speed a positive (uphill) trim fades out across the coast band: it never pushes the speed through it
    cruise_trim *= float(np.clip(1. + v_err / CRUISE_COAST_BAND, 0., 1.))
  target_accel = min(max(shaped_cruise_target(v_err, v_ego, max_accel) + cruise_trim, A_CRUISE_MIN), max_accel)
  j_cruise = float(np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS_SHAPED))
  j_down = j_cruise * (J_CRUISE_UNWIND_FACTOR if a_cruise_prev > 0. else 1.)
  j_up = J_CRUISE_RELEASE if a_cruise_prev < 0. else j_cruise
  return float(np.clip(target_accel, a_cruise_prev - j_down * dt, a_cruise_prev + j_up * dt))


def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle,
                     cruise_trim=0., shaped=False):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  if shaped:
    return shaped_cruise_accel(v_cruise - v_ego, v_ego, max_accel, a_cruise_prev, dt, cruise_trim)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.shaped_cruise = lexus_is_features.enabled("set_speed_law")
    self.cruise_trim = 0.0
    self.prev_plan_source = None
    self.cruise_trim_log_frame = 0
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target
      self.cruise_trim = 0.0

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    is_e2e = self.is_e2e(sm)

    a_cruise_prev = self.a_cruise
    if self.shaped_cruise:
      # The cruise candidate is computed every cycle, also while another candidate is lower. Do not let it ramp up out
      # of sight: start it no higher than the last output, so a mode switch or a departing lead cannot step the output
      # up to a candidate that was hidden. From a braking output it releases at J_CRUISE_RELEASE instead of jumping.
      a_cruise_prev = min(self.a_cruise, float(a_prev))
      # Slow trim of the steady error (grades): integrates only while the cruise law is in control near the set speed and
      # the car holds its speed; held while it accelerates or slows; decays when the cruise law is not in control.
      v_err = v_cruise - v_ego
      in_control = (not reset_state) and self.prev_plan_source == LongitudinalPlanSource.cruise and abs(v_err) < CRUISE_I_ZONE
      if in_control and abs(sm['carState'].aEgo) < CRUISE_I_STEADY_A:
        self.cruise_trim = float(np.clip(self.cruise_trim + CRUISE_KI * (v_err - CRUISE_APPROACH_MARGIN) * self.dt,
                                         -CRUISE_I_MAX, CRUISE_I_MAX))
      elif not in_control:
        self.cruise_trim *= max(0., 1. - self.dt / CRUISE_I_LEAK_TC)
      # the trim is not in longitudinalPlan: a 1 Hz record of it and the conditions it integrates under
      self.cruise_trim_log_frame += 1
      if self.cruise_trim_log_frame % CRUISE_TRIM_LOG_EVERY == 0:
        cloudlog.event("lexus_is_cruise_trim", trim=round(self.cruise_trim, 4), v_err=round(float(v_err), 3),
                       a_ego=round(float(sm['carState'].aEgo), 3), in_control=bool(in_control))
    self.a_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego,
                                     a_cruise_prev, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle, self.cruise_trim, self.shaped_cruise)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)
    if self.shaped_cruise and (v_cruise - v_ego) > CRUISE_APPROACH_MARGIN:
      # started from the last output, the candidate sits near 0 for a cycle or two at a standstill: never ask to stop
      # while the set speed is above the current speed
      cruise_should_stop = False

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if is_e2e:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.prev_plan_source = self.mpc.source
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
