import numpy as np
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from common.op_params import opParams


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

class PIDController:
  def __init__(self, k_p=0., k_i=0., k_d=0., k_f=1., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, derivative_period=1.):
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self.k_f = k_f  # feedforward gain
    self._k_d = k_d  # derivative gain
    
    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.sat_limit = sat_limit
    self._d_period = round(derivative_period * rate)  # period of time for derivative calculation (seconds converted to frames)

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.errors = []
    self.delayed_output = 0.

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p
    self.f = feedforward * self.k_f

    d = 0
    if len(self.errors) >= self._d_period:  # makes sure we have enough history for period
      d = (error - self.errors[-self._d_period]) / self._d_period  # get deriv in terms of 100hz (tune scale doesn't change)
      d *= self.k_d

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + self.f + i + d

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.f + self.i + d
    self.saturated = self._check_saturation(control, check_saturation, error)

    self.errors.append(float(error))
    while len(self.errors) > self._d_period:
      self.errors.pop(0)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
import numpy as np
import types
from numbers import Number
from common.numpy_fast import clip, interp
from common.op_params import opParams
from selfdrive.config import Conversions as CV


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


class PIDController:
  def __init__(self, k_p, k_i, k_d, k_f=1., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, derivative_period=1., convert=None, p_bp_key=None, p_v_key=None, i_bp_key=None, i_v_key=None,d_bp_key=None, d_v_key=None, f_key=None, sat_key=None, OP=None, use_ops=False):
    self._k_p = self._og_k_p = k_p  # proportional gain
    self._k_i = self._og_k_i = k_i  # integral gain
    self._k_d = self._og_k_d = k_d  # derivative gain
    self.k_f = self._og_k_f = k_f  # feedforward gain

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.sat_limit = self._og_sat_limit = sat_limit
    self._d_period = round(derivative_period * rate)  # period of time for derivative calculation (seconds converted to frames)
    self.convert = convert

    if OP is None:
      OP = opParams()

    self.op_params = OP
    self.p_bp_key = p_bp_key
    self.p_v_key = p_v_key
    self.i_bp_key = i_bp_key
    self.i_v_key = i_v_key
    self.d_bp_key = d_bp_key
    self.d_v_key = d_v_key
    self.f_key = f_key
    self.sat_key = sat_key
    self.use_ops = use_ops
    self.has_all_keys = p_bp_key and p_v_key and i_bp_key and i_v_key and d_bp_key and d_v_key and f_key and use_ops

    self.reset()

  def _can_use_ops(self):
    return self.has_all_keys and ((isinstance(self.use_ops, types.LambdaType) and self.use_ops(self.op_params)) or (isinstance(self.use_ops, str) and self.op_params.get(self.use_ops)))

  def _update_params(self):
    if self._can_use_ops():
      self._k_p = (self.op_params.get(self.p_bp_key), self.op_params.get(self.p_v_key))
      self._k_i = (self.op_params.get(self.i_bp_key), self.op_params.get(self.i_v_key))
      self._k_d = (self.op_params.get(self.d_bp_key), self.op_params.get(self.d_v_key))
      self.k_f = self.op_params.get(self.f_key)
      
      if self.sat_key:
        self.sat_limit = self.op_params.get(self.sat_key)
    else:
      self._k_p = self._og_k_p
      self._k_i = self._og_k_i
      self._k_d = self._og_k_d
      self._k_f = self._og_k_f
      self.sat_limit = self._og_sat_limit

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.errors = []
#    self.delayed_output = 0.

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self._update_params()
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p
    self.f = feedforward * self.k_f

    d = 0
    if len(self.errors) >= self._d_period:  # makes sure we have enough history for period
      d = (error - self.errors[-self._d_period]) / self._d_period  # get deriv in terms of 100hz (tune scale doesn't change)
      d *= self.k_d

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + self.f + i + d

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i
    control = self.p + self.i + d
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

#    alpha = 1. - DT_CTRL / (self.op_params.get('lat_rc') + DT_CTRL)
#    self.delayed_output = self.delayed_output * alpha + control * (1. - alpha)
#    control = float(self.delayed_output) + self.f
#
    self.saturated = self._check_saturation(control, check_saturation, error)

    self.errors.append(float(error))
    while len(self.errors) > self._d_period:
      self.errors.pop(0)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control