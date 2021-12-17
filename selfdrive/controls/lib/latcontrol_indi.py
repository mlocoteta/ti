import math
import numpy as np

from cereal import log
from common.filter_simple import FirstOrderFilter, Delay
from common.numpy_fast import clip
from common.realtime import DT_CTRL
from selfdrive.car import apply_ti_steer_torque_limits
from selfdrive.car.honda.values import CarControllerParams
from selfdrive.controls.lib.drive_helpers import get_steer_max

DEFAULT_G = 0.15
MAX_G = 1.0
MIN_G = 0.025

class LatControlINDI():
  def __init__(self, CP):
    self.angle_steers_des = 0.

    A = np.array([[1.0, DT_CTRL, 0.0],
                  [0.0, 1.0, DT_CTRL],
                  [0.0, 0.0, 1.0]])
    C = np.array([[1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0]])

    # Q = np.matrix([[1e-2, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 10.0]])
    # R = np.matrix([[1e-2, 0.0], [0.0, 1e3]])

    # (x, l, K) = control.dare(np.transpose(A), np.transpose(C), Q, R)
    # K = np.transpose(K)
    K = np.array([[7.30262179e-01, 2.07003658e-04],
                  [7.29394177e+00, 1.39159419e-02],
                  [1.71022442e+01, 3.38495381e-02]])

    self.K = K
    self.A_K = A - np.dot(K, C)
    self.x = np.array([[0.], [0.], [0.]])

    self.enforce_rate_limit = True

    self.sat_count_rate = 1.0 * DT_CTRL
    self.sat_limit = CP.steerLimitTimer

    # Actuator is modeled as a pure delay, followed by a first
    # order filter. Both with a time constant of CP.steerActuatorDelay
    self.steer_delay = Delay(0, CP.steerActuatorDelay, DT_CTRL)
    self.steer_filter = FirstOrderFilter(0., CP.steerActuatorDelay, DT_CTRL)

    self.steer_pressed_filter = FirstOrderFilter(0., 0.5, DT_CTRL)
    self.mu = 0.01
    self.G = DEFAULT_G

    self.reset()

  def reset(self):
    self.steer_filter.x = 0.
    self.steer_pressed_filter.x = 0.
    self.output_steer = 0.
    self.sat_count = 0.
    self.active_count = 0

  def _check_saturation(self, control, check_saturation, limit):
    saturated = abs(control) == limit

    if saturated and check_saturation:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def update(self, active, CS, CP, VM, params, curvature, curvature_rate):
    # Update Kalman filter
    y = np.array([[math.radians(CS.steeringAngleDeg)], [math.radians(CS.steeringRateDeg)]])
    self.x = np.dot(self.A_K, self.x) + np.dot(self.K, y)

    indi_log = log.ControlsState.LateralINDIState.new_message()
    indi_log.steeringAngleDeg = math.degrees(self.x[0])
    indi_log.steeringRateDeg = math.degrees(self.x[1])
    indi_log.steeringAccelDeg = math.degrees(self.x[2])

    steers_des = VM.get_steer_from_curvature(-curvature, CS.vEgo)
    steers_des += math.radians(params.angleOffsetDeg)
    indi_log.steeringAngleDesiredDeg = math.degrees(steers_des)

    rate_des = VM.get_steer_from_curvature(-curvature_rate, CS.vEgo)
    indi_log.steeringRateDesiredDeg = math.degrees(rate_des)

    if CS.vEgo < 0.3 or not active:
      indi_log.active = False
      self.output_steer = 0.0
      self.steer_filter.x = 0.0
      self.steer_delay.reset(0)
      self.steer_pressed_filter.x = 0.
      self.active_count = 0
    else:
      # Update actuator model with last steering output
      steer_filter_prev_x = self.steer_filter.x
      self.steer_filter.update(self.steer_delay.update(self.output_steer))
      self.active_count += 1

      if CS.steeringPressed:
        self.steer_pressed_filter.x = 1
      else:
        self.steer_pressed_filter.update(0)

      # Update effectiveness based on rate of change in control and angle
      pressed = self.steer_pressed_filter.x > 0.5
      engage_cooldown = self.active_count > 10 * self.steer_delay.n
      saturated = abs(self.output_steer) > 0.9
      if (not pressed) and engage_cooldown and (not saturated):
        delta_u = (self.steer_filter.x - steer_filter_prev_x) / DT_CTRL
        indi_log.accelSetPoint = float(delta_u)  # HACK
        self.G = self.G - self.mu * (self.G * delta_u - self.x[1]) * delta_u
        self.G = clip(self.G, MIN_G, MAX_G)

      # Compute desired change in actuator
      angle_error = steers_des - self.x[0]
      delta_u = angle_error / self.G

      # If steering pressed, only allow wind down
      if CS.steeringPressed and (delta_u * self.output_steer > 0):
        delta_u = 0
      self.params = CarControllerParams(CP)
      P = self.params
      # Enforce rate limit
      if self.enforce_rate_limit:
        steer_max = float(CarControllerParams.STEER_MAX)
        new_output_steer_cmd = steer_max * (self.steer_filter.x + delta_u)
        prev_output_steer_cmd = steer_max * self.output_steer
        new_output_steer_cmd = apply_ti_steer_torque_limits(new_output_steer_cmd, prev_output_steer_cmd, prev_output_steer_cmd, self.params)
        self.output_steer = new_output_steer_cmd / steer_max
      else:
        self.output_steer = self.steer_filter.x + delta_u

      steers_max = get_steer_max(CP, CS.vEgo)
      self.output_steer = clip(self.output_steer, -steers_max, steers_max)
      delta_u = self.output_steer - self.steer_filter.x

      indi_log.active = True
      indi_log.delayedOutput = float(self.steer_filter.x)
      indi_log.delta = float(delta_u)
      indi_log.output = float(self.output_steer)

      check_saturation = (CS.vEgo > 10.) and not CS.steeringRateLimited and not CS.steeringPressed
      indi_log.saturated = self._check_saturation(self.output_steer, check_saturation, steers_max)

    indi_log.accelError = float(self.G)  # HACK

    return float(self.output_steer), float(steers_des), indi_log