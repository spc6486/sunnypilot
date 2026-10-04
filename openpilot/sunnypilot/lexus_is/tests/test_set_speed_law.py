import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_planner import (A_CRUISE_MAX_BP, A_CRUISE_MIN, CRUISE_APPROACH_MARGIN,
                                                                    CRUISE_COAST_ACCEL, CRUISE_COAST_BAND, J_CRUISE_RELEASE,
                                                                    J_CRUISE_VALS, J_CRUISE_VALS_SHAPED, get_cruise_accel,
                                                                    shaped_cruise_accel, shaped_cruise_target)
from openpilot.common.realtime import DT_MDL

CP = type("CP", (), {"steerRatio": 13.3, "wheelbase": 2.8})()
MPH = 0.44704


def cruise(v_cruise, v_ego, a_prev, shaped, trim=0.):
  return get_cruise_accel(False, v_cruise, v_ego, a_prev, 0., CP, DT_MDL, 0., True, trim, shaped)


@pytest.mark.parametrize("v_ego", [0., 5., 15., 30.])
@pytest.mark.parametrize("v_err", [-3., -0.5, -0.1, 0., 0.3, 2., 8.])
@pytest.mark.parametrize("a_prev", [-1., 0., 0.8])
def test_off_is_upstream(v_ego, v_err, a_prev):
  # with the setting off, get_cruise_accel is upstream's clip + jerk limit
  max_accel = np.interp(v_ego, A_CRUISE_MAX_BP, [1.6, 1.2, 0.8, 0.6])
  j = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  want = float(np.clip(np.clip(v_err, A_CRUISE_MIN, max_accel), a_prev - j * DT_MDL, a_prev + j * DT_MDL))
  assert cruise(v_ego + v_err, v_ego, a_prev, shaped=False) == pytest.approx(want)


def test_target_shape():
  # approach aims below the set speed, holds 0 inside the margin, coasts in the band, brakes gently beyond it
  assert shaped_cruise_target(CRUISE_APPROACH_MARGIN, 20., 1.) == 0.
  assert shaped_cruise_target(1 * MPH, 26.8, 1.) == pytest.approx(0.3 * (MPH - CRUISE_APPROACH_MARGIN))
  assert shaped_cruise_target(-CRUISE_COAST_BAND, 20., 1.) == pytest.approx(CRUISE_COAST_ACCEL)
  assert CRUISE_COAST_ACCEL <= shaped_cruise_target(-0.2, 20., 1.) <= 0.
  assert shaped_cruise_target(-10., 20., 1.) == A_CRUISE_MIN
  assert shaped_cruise_target(20., 3., 1.6) == 1.6  # capped by the cruise acceleration table
  # monotonic in the speed error
  errs = np.linspace(-6., 6., 241)
  vals = [shaped_cruise_target(e, 20., 1.) for e in errs]
  assert all(b >= a for a, b in zip(vals, vals[1:], strict=False))


def test_one_mph_bumps_are_gentle():
  for d in (1 * MPH, -1 * MPH):
    a = 0.
    for _ in range(int(5 / DT_MDL)):
      a = shaped_cruise_accel(d, 26.8, 0.8, a, DT_MDL, 0.)
    assert abs(a) < 0.15


def test_rates():
  # rises at the shaped jerk table, positive acceleration unwinds at twice that, negative releases at J_CRUISE_RELEASE
  j = np.interp(10., A_CRUISE_MAX_BP, J_CRUISE_VALS_SHAPED)
  assert shaped_cruise_accel(10., 10., 2., 0.5, DT_MDL, 0.) == pytest.approx(0.5 + j * DT_MDL)
  assert shaped_cruise_accel(0., 10., 2., 0.5, DT_MDL, 0.) == pytest.approx(0.5 - 2 * j * DT_MDL)
  assert shaped_cruise_accel(10., 10., 2., -1., DT_MDL, 0.) == pytest.approx(-1. + J_CRUISE_RELEASE * DT_MDL)


def test_trim_adds_and_is_bounded():
  assert shaped_cruise_accel(0.05, 20., 1., 0.2, DT_MDL, 0.2) == pytest.approx(0.2)
  assert shaped_cruise_target(-10., 20., 1.) + (-0.3) < A_CRUISE_MIN
  assert shaped_cruise_accel(-10., 20., 1., A_CRUISE_MIN, DT_MDL, -0.3) == A_CRUISE_MIN


def test_positive_trim_fades_above_the_set_speed():
  # a positive (uphill) trim adds fully at or below the set speed, fades across the coast band, and is gone beyond it
  def hold(v_err, trim):
    return shaped_cruise_accel(v_err, 25., 1., 0., 1e3, trim)  # large dt: no rate limit
  assert hold(0., 0.3) == pytest.approx(shaped_cruise_target(0., 25., 1.) + 0.3)
  half = -CRUISE_COAST_BAND / 2
  assert hold(half, 0.3) == pytest.approx(shaped_cruise_target(half, 25., 1.) + 0.15)
  assert hold(-CRUISE_COAST_BAND, 0.3) == pytest.approx(shaped_cruise_target(-CRUISE_COAST_BAND, 25., 1.))
  assert hold(-1., 0.3) == pytest.approx(shaped_cruise_target(-1., 25., 1.))
  # a negative (downhill) trim is not faded above the set speed
  assert hold(-1., -0.2) == pytest.approx(shaped_cruise_target(-1., 25., 1.) - 0.2)
