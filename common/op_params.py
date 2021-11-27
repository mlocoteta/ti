#!/usr/bin/env python3
import os
import json
import math
import time
from common.numpy_fast import find_nearest_index, interp, is_multi_iter
from common.colors import opParams_error as error
from common.colors import opParams_warning as warning
from selfdrive.hardware import PC
try:
  from common.realtime import sec_since_boot
except ImportError:
  sec_since_boot = time.time
  warning("Using python time.time() instead of faster sec_since_boot")

travis = True if PC else False  # replace with travis_checker if you use travis or GitHub Actions


def parse_param_modifiers(src, value):
  if src:
    if ParamModifierKeys.ABS in src:
      return parse_param_modifiers(src.replace(ParamModifierKeys.ABS, ''), abs(value))
    elif ParamModifierKeys.DEGREES in src:
      return parse_param_modifiers(src.replace(ParamModifierKeys.DEGREES, ''), math.degrees(value))
    elif ParamModifierKeys.RADIANS in src:
      return parse_param_modifiers(src.replace(ParamModifierKeys.RADIANS, ''), math.radians(value))
  else:
    return value

def eval_breakpoint_source(sources, CS, controls_state):
  '''
  Maps a breakpoint source array to actual values
  '''

  def eval_source(src):
    if BreakPointSourceKeys.VEGO in src:
      return parse_param_modifiers(src.replace(BreakPointSourceKeys.VEGO, ''), CS.vEgo)
    elif BreakPointSourceKeys.AEGO in src:
      return parse_param_modifiers(src.replace(BreakPointSourceKeys.AEGO, ''), CS.aEgo)
    elif BreakPointSourceKeys.DESIRED_STEER in src:
      return parse_param_modifiers(src.replace(BreakPointSourceKeys.DESIRED_STEER, ''), controls_state.desiredSteerDeg)
    else:
      raise ValueError(f'Unknown value option: {src}')

  return [eval_source(source) for source in sources]

def interp_multi_bp(x, bp, v):
  def correct_multi_bp(idx):
    if not is_multi_iter(bp[idx]):
      bp[idx] = [bp[idx], bp[idx]]

      if len(bp) <= 1:
        bp.insert(0, bp[idx][0])

  is_bp_multi_iter = is_multi_iter(bp)
  is_v_multi_iter = is_multi_iter(v)

  if not is_bp_multi_iter:
    bp = [bp, bp]

  # correct_multi_bp(0)
  correct_multi_bp(-1)

  if not is_v_multi_iter:
    v = [v, v]

  l_x = len(x)
  l_bp = len(bp)
  l_v = len(v)

  # print(f'bp: {bp}')
  if l_v <= 1:
    v = [v[-1], v[-1]]

  if l_bp < l_x or not hasattr(bp[0], '__iter__') or len(bp[0]) <= 1:
    # return interp(x[0], bp[0][0], v[0])
    # idx = range(len(x)) if is_multi_iter(x) else 0
    # idx = [0] if is_multi_iter(x) else 0
    idx = 0
  else:
    idx = find_nearest_index(bp[0], x[0])

  # print(f'indexes: {idx}')

  if hasattr(idx, '__iter__'):
    return [interp(x[-1], bp[-1][-1], v[min(l_v - 1, i)]) for i in set(idx)]
  else:
    return interp(x[-1], bp[-1][-1], v[min(l_v - 1, idx)])

  # return [interp(x[-1], bp[-1][i], v[i]) for i in set(idx)] if hasattr(idx, '__iter__') else interp(x[-1], bp[-1][idx], v[idx])
  # return interp(x[1], bp[1][idx], v[idx])

class BreakPointSourceKeys:
  VEGO = 'vego'
  AEGO = 'aego'
  DESIRED_STEER = 'desired_steer'

class ParamModifierKeys:
  ABS = '_abs'
  DEGREES = '_deg'
  RADIANS = '_rad'

class ValueTypes:
  number = [float, int]
  none_or_number = [type(None), float, int]
  list_of_numbers = [list, float, int]

class Param:
  def __init__(self, default, allowed_types, description=None, live=False, hidden=False, depends_on=None):
    self.default = default
    if not isinstance(allowed_types, list):
      allowed_types = [allowed_types]
    self.allowed_types = allowed_types
    self.description = description
    self.hidden = hidden
    self.live = live
    self.depends_on = depends_on
    self.children = []
    self._create_attrs()

  def is_valid(self, value):
    if not self.has_allowed_types:
      return True
    if self.is_list and isinstance(value, list):
      for v in value:
        if type(v) not in self.allowed_types:
          return False
      return True
    else:
      return type(value) in self.allowed_types or value in self.allowed_types

  def _create_attrs(self):  # Create attributes and check Param is valid
    self.has_allowed_types = isinstance(self.allowed_types, list) and len(self.allowed_types) > 0
    self.has_description = self.description is not None
    self.is_list = list in self.allowed_types
    self.is_bool = bool in self.allowed_types
    if self.has_allowed_types:
      assert type(self.default) in self.allowed_types or self.default in self.allowed_types, 'Default value type must be in specified allowed_types!'

      if self.is_list and self.default:
        for v in self.default:
          assert type(v) in self.allowed_types, 'Default value type must be in specified allowed_types!'


class opParams:
  def __init__(self):
    """
      To add your own parameter to opParams in your fork, simply add a new entry in self.fork_params, instancing a new Param class with at minimum a default value.
      The allowed_types and description args are not required but highly recommended to help users edit their parameters with opEdit safely.
        - The description value will be shown to users when they use opEdit to change the value of the parameter.
        - The allowed_types arg is used to restrict what kinds of values can be entered with opEdit so that users can't crash openpilot with unintended behavior.
              (setting a param intended to be a number with a boolean, or viceversa for example)
          Limiting the range of floats or integers is still recommended when `.get`ting the parameter.
          When a None value is allowed, use `type(None)` instead of None, as opEdit checks the type against the values in the arg with `isinstance()`.
        - Finally, the live arg tells both opParams and opEdit that it's a live parameter that will change. Therefore, you must place the `op_params.get()` call in the update function so that it can update.

      Here's an example of a good fork_param entry:
      self.fork_params = {'camera_offset': Param(default=0.06, allowed_types=VT.number)}  # VT.number allows both floats and ints
    """

    VT = ValueTypes()
    self.fork_params = {#CAM_OFFSET: Param(0.06, VT.number, 'Your camera offset to use in lane_planner.py', live=True),
                        'indi_inner_gain': Param(9.0, VT.number, live=True, depends_on=SHOW_INDI_PARAMS),
                        'indi_outer_gain': Param(8.9, VT.number, live=True, depends_on=SHOW_INDI_PARAMS),
                        'indi_time_constant': Param(5.5, VT.number, live=True, depends_on=SHOW_INDI_PARAMS),
                        'indi_actuator_effectiveness': Param(9.0, VT.number, live=True, depends_on=SHOW_INDI_PARAMS),
                        SHOW_ACTUATOR_DELAY_PARAMS: Param(False, bool, live=True, depends_on=ENABLE_LAT_PARAMS),
                        STEER_ACTUATOR_DELAY: Param(0.60, VT.number, live=True, depends_on=SHOW_ACTUATOR_DELAY_PARAMS),
                        ENABLE_ACTUATOR_DELAY_BPS: Param(False, bool, live=True, depends_on=SHOW_ACTUATOR_DELAY_PARAMS),
                        STEER_ACTUATOR_DELAY_BP: Param([0.], [list, float, int], live=True, depends_on=ENABLE_ACTUATOR_DELAY_BPS),
                        STEER_ACTUATOR_DELAY_V: Param([0.6], [list, float, int], live=True, depends_on=ENABLE_ACTUATOR_DELAY_BPS),
#                        ENABLE_ACTUATOR_DELAY_BPS_MULTI: Param(False, bool, live=True, depends_on=SHOW_ACTUATOR_DELAY_PARAMS),
#                        STEER_ACTUATOR_DELAY_BP_MULTI: Param([[0], [0, 4, 9, 17]], [list, float, int], live=True, depends_on=ENABLE_ACTUATOR_DELAY_BPS_MULTI),
#                        STEER_ACTUATOR_DELAY_V_MULTI: Param([[0.45, 0.4, 0.3, 0.16]], [list, float, int], live=True, depends_on=ENABLE_ACTUATOR_DELAY_BPS_MULTI),
#                        STEER_DELAY_MULTI_BP_SOURCE: Param(['vego', 'desired_steer_abs'], [list, str], live=True, depends_on=ENABLE_ACTUATOR_DELAY_BPS_MULTI),
                        ENABLE_INDI_BREAKPOINTS: Param(False, bool, live=True, depends_on=SHOW_INDI_PARAMS),
                        INDI_INNER_GAIN_BP: Param([20, 24, 30], [list, float, int], live=True, depends_on=ENABLE_INDI_BREAKPOINTS),
                        INDI_INNER_GAIN_V: Param([7.25, 7.5, 9], [list, float, int], live=True, depends_on=ENABLE_INDI_BREAKPOINTS),
                        INDI_OUTER_GAIN_BP: Param([20, 24, 30], [list, float, int], live=True, depends_on=ENABLE_INDI_BREAKPOINTS),
                        INDI_OUTER_GAIN_V: Param([6, 7.25, 6], [list, float, int], live=True, depends_on=ENABLE_INDI_BREAKPOINTS),
                        INDI_TIME_CONSTANT_BP: Param([20, 24], [list, float, int], live=True, depends_on=ENABLE_INDI_BREAKPOINTS),
                        INDI_TIME_CONSTANT_V: Param([1.6, 1.83], [list, float, int], live=True, depends_on=ENABLE_INDI_BREAKPOINTS),
                        INDI_ACTUATOR_EFFECTIVENESS_BP: Param([0, 24], [list, float, int], live=True, depends_on=ENABLE_INDI_BREAKPOINTS),
                        INDI_ACTUATOR_EFFECTIVENESS_V: Param([2, 3], [list, float, int], live=True, depends_on=ENABLE_INDI_BREAKPOINTS),
                        ENABLE_LAT_PARAMS: Param(False, bool, live=True, description="When true, the lat params set in op_edit."),
                        WHICH_LAT_CTRL: Param('indi', ['pid', 'indi'], live=True, depends_on= ENABLE_LAT_PARAMS, description='Which lat controller to use, '
                                              'options are pid or indi'),
                        STEER_LIMIT_TIMER: Param(0.4, VT.number, live=True, depends_on=ENABLE_LAT_PARAMS),
                        SHOW_LAT_PID_PARAMS: Param(False, [bool], live=True, depends_on=ENABLE_LAT_PARAMS),
                        LAT_PID_KP_BP: Param([0., 5., 35.], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KP_V: Param([3.6, 2.4, 1.5], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KI_BP: Param([0., 35.], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KI_V: Param([0.54, 0.36], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KD_BP: Param([0., 35.], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KD_V: Param([0.54, 0.36], [list, float, int], live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        LAT_PID_KF: Param(1., VT.number, live=True, depends_on=SHOW_LAT_PID_PARAMS),
                        SHOW_INDI_PARAMS: Param(False, [bool], live=True, depends_on=ENABLE_LAT_PARAMS),
                        SHOW_RATE_PARAMS: Param(False, [bool], live=True, depends_on=ENABLE_LAT_PARAMS),
                        ENABLE_RATE_PARAMS: Param(False, [bool], live=True, depends_on=SHOW_RATE_PARAMS),
                        TI_STEER_MAX: Param(600, VT.number, live=True, depends_on=SHOW_RATE_PARAMS),
                        TI_STEER_DELTA_UP: Param(6, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
                        TI_STEER_DELTA_UP_LOW: Param(6, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
                        TI_STEER_DELTA_DOWN: Param(15, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
                        TI_STEER_DELTA_DOWN_LOW: Param(15, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
                        TI_HIGH_BP: Param(150, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
                        STOCK_DELTA_UP: Param(7, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
                        STOCK_DELTA_DOWN: Param(14, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
                        STOCK_STEER_MAX: Param(238, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
                        TI_JUMPING_POINT: Param(0, VT.number, live=True ,depends_on=SHOW_RATE_PARAMS),
#                        SHOW_UNSAFE_OPTS: Param(False, [bool], live=True, description='Shows options for unsafe / dangerous features. '
#                                                'If any of these are enabled, prepare for the worst: no steering, no gas / brake, etc.'),
#                        SHOW_EXPERIMENTAL_OPTS: Param(False, [bool], live=True, description='Shows options for experimental, unfinished, features. '
#                                                      'Generally you should never use these.'),
#                        SHOW_TOYOTA_OPTS: Param(False, [bool], live=True, description='Shows options toyota cars.'),
#                        COROLLA_BODY_TYPE: Param('hatchback', ['sedan', 'hatchback'], depends_on=SHOW_TOYOTA_OPTS),
#                        ENABLE_MANAGER_PARAMS: Param(False, [bool], depends_on=SHOW_UNSAFE_OPTS),
#                        DISABLED_PROCESSES: Param(None, [str, list, type(None)], description='You\'re on your own here', depends_on=ENABLE_MANAGER_PARAMS),
#                        ENABLE_TOYOTA_CAN_PARAMS: Param(False, [bool], live=True, depends_on=SHOW_TOYOTA_OPTS),
#                        ENABLE_TOYOTA_ACCEL_PARAMS: Param(False, [bool], live=True, depends_on=ENABLE_TOYOTA_CAN_PARAMS),
#                        TOYOTA_ACC_TYPE: Param(1, [int], live=True, depends_on=ENABLE_TOYOTA_ACCEL_PARAMS),
#                        TOYOTA_PERMIT_BRAKING: Param(1, [1, 0, 'lead'], live=True, depends_on=ENABLE_TOYOTA_ACCEL_PARAMS),
                        ENABLE_PLANNER_PARAMS: Param(False, [bool], live=True),
#                        ENABLE_PLNR_ACCEL_LIMITS: Param(False, [bool], live=True, depends_on=ENABLE_PLANNER_PARAMS),
#                        'a_cruise_min_bp': Param([0., 5.,  10., 20.,  40.], [list, float], live=True, depends_on=ENABLE_PLNR_ACCEL_LIMITS),
#                        'a_cruise_min_v': Param([-1.0, -.8, -.67, -.5, -.30], [list, float], live=True, depends_on=ENABLE_PLNR_ACCEL_LIMITS),
#                        'a_cruise_min_v_following': Param([-1.0, -.8, -.67, -.5, -.30], [list, float], live=True, depends_on=ENABLE_PLNR_ACCEL_LIMITS),
#                        'a_cruise_max_bp': Param([0.,  6.4, 22.5, 40.], [list, float], live=True, depends_on=ENABLE_PLNR_ACCEL_LIMITS),
#                        'a_cruise_max_v': Param([1.2, 1.2, 0.65, .4], [list, float], live=True, depends_on=ENABLE_PLNR_ACCEL_LIMITS),
#                        'a_cruise_max_v_following': Param([1.6, 1.6, 0.65, .4], [list, float], live=True, depends_on=ENABLE_PLNR_ACCEL_LIMITS),
                        ENABLE_STEER_RATE_COST: Param(False, [bool], live=True, depends_on=ENABLE_PLANNER_PARAMS, description='Technically live but treat it like it\'s not.'),
                        STEER_RATE_COST: Param(0.5, VT.number, live=True, depends_on=ENABLE_STEER_RATE_COST),
#                        SHOW_BUILD_OPTS: Param(False, [bool], live=False, description='Show options for compile time features.'),
#                        ENABLE_SCREEN_BRIGHTNESS_HEAD_LIGHTS: Param(False, [bool], depends_on=SHOW_BUILD_OPTS, description='When enabled, the screen brightness will adjust depending on the car headlights.'),
#                        DAY_BRIGHTNESS: Param(245, VT.number, depends_on=ENABLE_SCREEN_BRIGHTNESS_HEAD_LIGHTS),
#                        NIGHT_BRIGHTNESS: Param(50, VT.number, depends_on=ENABLE_SCREEN_BRIGHTNESS_HEAD_LIGHTS),
#                        HIGH_BEAM_BRIGHTNESS: Param(20, VT.number, depends_on=ENABLE_SCREEN_BRIGHTNESS_HEAD_LIGHTS),
#                        DISENGAGE_ON_GAS: Param(True, [bool], description='Whether you want openpilot to disengage on gas input or not.', live=True),
#                        ENABLE_ROAD_SIGNS: Param(False, [bool], live=True, depends_on=SHOW_TOYOTA_OPTS, description='Use Toyota\'s road sign assist to control OP speed.'),
                        ENABLE_CURVE_RATE_LIMITS: Param(False, [bool], live=True, depends_on=ENABLE_LAT_PARAMS, description='Override the default max curvature rates when true.'),
                        MAX_CURVE_RATE_BP: Param([0, 35], VT.list_of_numbers, live=True, depends_on=ENABLE_CURVE_RATE_LIMITS),
                        MAX_CURVE_RATE_V: Param([0.03762194918267951, 0.003441203371932992], VT.list_of_numbers, live=True, depends_on=ENABLE_CURVE_RATE_LIMITS,
                                                description='Default values corresponds to 80deg/s and 20deg/s steering angle in a toyota corolla.'),
#                        ENABLE_MODEL_PARAMS: Param(False, [bool], description='Apply model params / options when true. Compile time only.'),
#                        USE_OLD_DESIRE_MODEL: Param(False, [bool], description='Use the old desire model that didn\'t make it into OP v0.8.6. Compile time only.', depends_on=ENABLE_MODEL_PARAMS),
                        API_URL: Param('api.commadotai.com', [str], description='Domain to use for the API. Domain name only, don\'t include https or extra slashes.')}

    self._params_file = '/data/op_params.json'
    self._backup_file = '/data/op_params_corrupt.json'
    self._last_read_time = sec_since_boot()
    self.read_frequency = 2.5  # max frequency to read with self.get(...) (sec)
    self._to_delete = ['lane_hug_direction', 'lane_hug_angle_offset', 'prius_use_lqr']  # a list of params you want to delete (unused)
    self._last_mod_time = 0.0
    self._run_init()  # restores, reads, and updates params

  def _run_init(self):  # does first time initializing of default params
    # Two required parameters for opEdit
    self.fork_params['username'] = Param(None, [type(None), str, bool], 'Your identifier provided with any crash logs sent to Sentry.\nHelps the developer reach out to you if anything goes wrong')
    self.fork_params['op_edit_live_mode'] = Param(False, bool, 'This parameter controls which mode opEdit starts in', hidden=True)
    self.params = self._get_all_params(default=True)  # in case file is corrupted

    for k, p in self.fork_params.items():
      d = p.depends_on
      while d:
        fp = self.fork_params[d]
        fp.children.append(k)
        d = fp.depends_on

    if travis:
      return

    to_write = False
    if os.path.isfile(self._params_file):
      if self._read():
        to_write = self._add_default_params()  # if new default data has been added
        to_write |= self._delete_old()  # or if old params have been deleted
      else:  # backup and re-create params file
        error("Can't read op_params.json file, backing up to /data/op_params_corrupt.json and re-creating file!")
        to_write = True
        if os.path.isfile(self._backup_file):
          os.remove(self._backup_file)
        os.rename(self._params_file, self._backup_file)
    else:
      to_write = True  # user's first time running a fork with op_params, write default params

    if to_write:
      if self._write():
        os.chmod(self._params_file, 0o764)

  def get(self, key=None, force_live=False):  # any params you try to get MUST be in fork_params
    if PC:
      assert isinstance(self, opParams), f'Self is type: {type(self).__name__}, expected opParams'

    param_info = self.param_info(key)
    self._update_params(param_info, force_live)

    if key is None:
      return self._get_all_params()

    self._check_key_exists(key, 'get')
    value = self.params[key]
    if param_info.is_valid(value):  # always valid if no allowed types, otherwise checks to make sure
      return value  # all good, returning user's value

    warning('User\'s value type is not valid! Returning default')  # somehow... it should always be valid
    return param_info.default  # return default value because user's value of key is not in allowed_types to avoid crashing openpilot

  def put(self, key, value):
    self._check_key_exists(key, 'put')
    if not self.param_info(key).is_valid(value):
      raise Exception('opParams: Tried to put a value of invalid type!')
    self.params.update({key: value})
    self._write()

  def delete(self, key):  # todo: might be obsolete. remove?
    if key in self.params:
      del self.params[key]
      self._write()

  def param_info(self, key):
    if key in self.fork_params:
      return self.fork_params[key]
    return Param(None, type(None))

  def _check_key_exists(self, key, met):
    if key not in self.fork_params or key not in self.params:
      raise Exception('opParams: Tried to {} an unknown parameter! Key not in fork_params: {}'.format(met, key))

  def _add_default_params(self):
    added = False
    for key, param in self.fork_params.items():
      if key not in self.params:
        self.params[key] = param.default
        added = True
      elif not param.is_valid(self.params[key]):
        warning('Value type of user\'s {} param not in allowed types, replacing with default!'.format(key))
        self.params[key] = param.default
        added = True
    return added

  def _delete_old(self):
    deleted = False
    for param in self._to_delete:
      if param in self.params:
        del self.params[param]
        deleted = True
    return deleted

  def _get_all_params(self, default=False, return_hidden=False):
    if default:
      return {k: p.default for k, p in self.fork_params.items()}
    return {k: self.params[k] for k, p in self.fork_params.items() if k in self.params and (not p.hidden or return_hidden)}

  def _update_params(self, param_info, force_live):
    if force_live or param_info.live:  # if is a live param, we want to get updates while openpilot is running
      if not travis and sec_since_boot() - self._last_read_time >= self.read_frequency:  # make sure we aren't reading file too often
        if self._read():
          self._last_read_time = sec_since_boot()

  def _read(self):
    if os.path.isfile(self._params_file):
      try:
        mod_time = os.path.getmtime(self._params_file)
        if mod_time > self._last_mod_time:
          with open(self._params_file, "r") as f:
            self.params = json.loads(f.read())
          self._last_mod_time = mod_time
          return True
        else:
          return False
      except Exception as e:
        print("Unable to read file: " + str(e))
        return False
    else:
      return False

  def _write(self):
    if not travis or os.path.isdir("/data/"):
      try:
        with open(self._params_file, "w") as f:
          f.write(json.dumps(self.params, indent=2))  # can further speed it up by remove indentation but makes file hard to read
        return True
      except Exception as e:
        print("Unable to write file: " + str(e))
        return False

#CAM_OFFSET = 'camera_offset'

#ENABLE_UNSAFE_STEERING_RATE = "enable_unsafe_steering_rate"
#ENABLE_UNSAFE_STEERING_RATE_SELFDRIVE = "enable_unsafe_steering_rate_selfdrive"

#ENABLE_COASTING = "enable_coasting"
#COAST_SPEED = "coast_speed"
#SETPOINT_OFFSET = "setpoint_offset"
#DOWNHILL_INCLINE = "downhill_incline"
#ALWAYS_EVAL_COAST = "always_eval_coast_plan"
#EVAL_COAST_LONG = "eval_coast_long_controller"
SHOW_RATE_PARAMS = 'show_rate_params'
ENABLE_RATE_PARAMS = 'enable_rate_params'
TI_STEER_MAX = 'ti_steer_max'
TI_STEER_DELTA_UP = 'ti_steer_delta_up'
TI_STEER_DELTA_UP_LOW = 'ti_steer_delta_up_low'
TI_STEER_DELTA_DOWN = 'ti_steer_delta_down'
TI_STEER_DELTA_DOWN_LOW = 'ti_steer_delta_down_low'
TI_HIGH_BP = 'ti_high_bp'
STOCK_DELTA_UP = 'stock_delta_up'
STOCK_DELTA_DOWN = 'stock_delta_down'
STOCK_STEER_MAX = 'stock_steer_max'
TI_JUMPING_POINT = 'ti_jumping_point'

SHOW_INDI_PARAMS = 'show_indi_params'
ENABLE_INDI_BREAKPOINTS = 'enable_indi_breakpoints'
INDI_INNER_GAIN_BP = 'indi_inner_gain_bp'
INDI_INNER_GAIN_V = 'indi_inner_gain_v'
INDI_OUTER_GAIN_BP = 'indi_outer_gain_bp'
INDI_OUTER_GAIN_V = 'indi_outer_gain_v'
INDI_ACTUATOR_EFFECTIVENESS_BP = 'indi_actuator_effectiveness_bp'
INDI_ACTUATOR_EFFECTIVENESS_V = 'indi_actuator_effectiveness_v'
INDI_TIME_CONSTANT_BP = 'indi_time_constant_bp'
INDI_TIME_CONSTANT_V = 'indi_time_constant_v'
#ENABLE_MULTI_INDI_BREAKPOINTS = 'enable_multi_indi_breakpoints'
#INDI_INNER_GAIN_BP_MULTI = 'indi_inner_gain_bp_multi'
#INDI_INNER_GAIN_V_MULTI = 'indi_inner_gain_v_multi'
#INDI_OUTER_GAIN_BP_MULTI = 'indi_outer_gain_bp_multi'
#INDI_OUTER_GAIN_V_MULTI = 'indi_outer_gain_v_multi'
#INDI_ACTUATOR_EFFECTIVENESS_BP_MULTI = 'indi_actuator_effectiveness_bp_multi'
#INDI_ACTUATOR_EFFECTIVENESS_V_MULTI = 'indi_actuator_effectiveness_v_multi'
#INDI_TIME_CONSTANT_BP_MULTI = 'indi_time_constant_bp_multi'
#INDI_TIME_CONSTANT_V_MULTI = 'indi_time_constant_v_multi'
#INDI_MULTI_BREAKPOINT_SOURCE = 'indi_multi_breakpoint_source'

#SHOW_A_CRUISE = 'a_cruise_show_opts'

#ENABLE_LONG_PARAMS = 'enable_long_params'
#ENABLE_GAS_PARAMS = 'enable_gas_params'
#GAS_MAX_BP = 'gas_max_bp'
#GAS_MAX_V = 'gas_max_v'
#ENABLE_BRAKE_PARAMS = 'enable_brake_params'
#BRAKE_MAX_BP = 'brake_max_bp'
#BRAKE_MAX_V = 'brake_max_v'
#ENABLE_LONG_PID_PARAMS = 'enable_long_pid_params'
#LONG_PID_KP_BP = 'long_pid_kp_bp'
#LONG_PID_KP_V = 'long_pid_kp_v'
#LONG_PID_KI_BP = 'long_pid_ki_bp'
#LONG_PID_KI_V = 'long_pid_ki_v'
#LONG_PID_KF = 'long_pid_kf'
#LONG_PID_SAT_LIMIT = 'long_pid_sat_limit'
#ENABLE_LONG_DEADZONE_PARAMS = 'enable_long_deadzone_params'
#LONG_DEADZONE_BP = 'long_deadzone_bp'
#LONG_DEADZONE_V = 'long_deadzone_v'
#ENABLE_START_STOP_PARAMS = 'enable_start_stop_params'
#STOP_BRAKE_RATE_BP = 'stopping_brake_rate_bp'
#STOP_BRAKE_RATE_V = 'stopping_brake_rate_v'
#START_BRAKE_RATE_BP = 'starting_brake_rate_bp'
#START_BRAKE_RATE_V = 'starting_brake_rate_v'

ENABLE_LAT_PARAMS = 'enable_lat_params'
WHICH_LAT_CTRL = 'which_lat_controller'

#SHOW_LQR_PARAMS = 'show_lqr_params'
#LQR_SCALE = 'lqr_scale'
#LQR_KI = 'lqr_ki'
#LQR_A = 'lqr_a'
#LQR_B = 'lqr_b'
#LQR_C = 'lqr_c'
#LQR_K = 'lqr_k'
#LQR_L = 'lqr_l'
#LQR_DC_GAIN = 'lqr_dc_gain'
STEER_LIMIT_TIMER = 'steer_limit_timer'

SHOW_ACTUATOR_DELAY_PARAMS = "show_actuator_delay_params"
STEER_ACTUATOR_DELAY = 'steer_actuator_delay'
ENABLE_ACTUATOR_DELAY_BPS = 'enable_actuator_delay_breakpoints'
STEER_ACTUATOR_DELAY_BP = 'steer_actuator_delay_bp'
STEER_ACTUATOR_DELAY_V = 'steer_actuator_delay_v'
#ENABLE_ACTUATOR_DELAY_BPS_MULTI = 'enable_actuator_delay_breakpoints_multi'
#STEER_ACTUATOR_DELAY_BP_MULTI = 'steer_actuator_delay_bp_multi'
#STEER_ACTUATOR_DELAY_V_MULTI = 'steer_actuator_delay_v_multi'
#STEER_DELAY_MULTI_BP_SOURCE = 'steer_actuator_delay_multi_bp_source'

SHOW_LAT_PID_PARAMS = 'show_lat_pid_params'
LAT_PID_KP_BP = 'lat_pid_kp_bp'
LAT_PID_KP_V = 'lat_pid_kp_v'
LAT_PID_KI_BP = 'lat_pid_ki_bp'
LAT_PID_KI_V = 'lat_pid_ki_v'
LAT_PID_KD_BP = 'lat_pid_kd_bp'
LAT_PID_KD_V = 'lat_pid_kd_v'
LAT_PID_KF = 'lat_pid_kf'


#SHOW_UNSAFE_OPTS = 'show_unsafe_options'
#SHOW_EXPERIMENTAL_OPTS = 'show_experimental_options'

#SHOW_TOYOTA_OPTS = 'show_toyota_options'
#COROLLA_BODY_TYPE = 'corolla_body_type'
#ENABLE_ACCEL_HYST_GAP = 'enable_accel_hyst_gap'
#ACCEL_HYST_GAP = 'accel_hyst_gap'
#ENABLE_ACCEL_HYST_GAP_BPS = 'enable_accel_hyst_gap_breakpoints'
#ACCEL_HYST_GAP_BP = 'accel_hyst_gap_bp'
#ACCEL_HYST_GAP_V = 'accel_hyst_gap_v'

#ENABLE_MANAGER_PARAMS = 'enable_manager_params'
#DISABLED_PROCESSES = 'disabled_processes'

#ENABLE_TOYOTA_CAN_PARAMS = 'enable_toyota_can_params'
#ENABLE_TOYOTA_ACCEL_PARAMS = 'enable_toyota_accel_params'
#TOYOTA_ACC_TYPE = 'toyota_acc_type'
#TOYOTA_PERMIT_BRAKING = 'toyota_permit_braking'

ENABLE_PLANNER_PARAMS = 'enable_planner_params'
#ENABLE_PLNR_ACCEL_LIMITS = 'enable_accel_limits_planner'
ENABLE_STEER_RATE_COST = 'enable_steer_rate_cost'
STEER_RATE_COST = 'steer_rate_cost'

#SHOW_BUILD_OPTS = 'show_build_options'
#ENABLE_SCREEN_BRIGHTNESS_HEAD_LIGHTS = 'enable_screen_brightness_head_lights'
#DAY_BRIGHTNESS = 'day_time_brightness'
#NIGHT_BRIGHTNESS = 'night_time_brightness'
#HIGH_BEAM_BRIGHTNESS = 'high_beam_brightness'

#DISENGAGE_ON_GAS = 'disengage_on_gas'

#ENABLE_ROAD_SIGNS = 'enable_road_sign_assist'

ENABLE_CURVE_RATE_LIMITS = 'enable_curvature_rate_limits'
MAX_CURVE_RATE_BP = 'max_curvature_rate_bp'
MAX_CURVE_RATE_V = 'max_curvature_rate_v'

#ENABLE_MODEL_PARAMS = 'enable_model_params'
#USE_OLD_DESIRE_MODEL = 'use_old_new_desire_model'

API_URL = 'api_url'
