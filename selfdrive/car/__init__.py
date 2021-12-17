# functions common among cars
from cereal import car
from common.numpy_fast import clip

# kg of standard extra cargo to count for drive, gas, etc...
STD_CARGO_KG = 136.


def gen_empty_fingerprint():
  return {i: {} for i in range(0, 4)}


# FIXME: hardcoding honda civic 2016 touring params so they can be used to
# scale unknown params for other cars
class CivicParams:
  MASS = 1326. + STD_CARGO_KG
  WHEELBASE = 2.70
  CENTER_TO_FRONT = WHEELBASE * 0.4
  CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT
  ROTATIONAL_INERTIA = 2500
  TIRE_STIFFNESS_FRONT = 192150
  TIRE_STIFFNESS_REAR = 202500


# TODO: get actual value, for now starting with reasonable value for
# civic and scaling by mass and wheelbase
def scale_rot_inertia(mass, wheelbase):
  return CivicParams.ROTATIONAL_INERTIA * mass * wheelbase ** 2 / (CivicParams.MASS * CivicParams.WHEELBASE ** 2)


# TODO: start from empirically derived lateral slip stiffness for the civic and scale by
# mass and CG position, so all cars will have approximately similar dyn behaviors
def scale_tire_stiffness(mass, wheelbase, center_to_front, tire_stiffness_factor=1.0):
  center_to_rear = wheelbase - center_to_front
  tire_stiffness_front = (CivicParams.TIRE_STIFFNESS_FRONT * tire_stiffness_factor) * mass / CivicParams.MASS * \
                         (center_to_rear / wheelbase) / (CivicParams.CENTER_TO_REAR / CivicParams.WHEELBASE)

  tire_stiffness_rear = (CivicParams.TIRE_STIFFNESS_REAR * tire_stiffness_factor) * mass / CivicParams.MASS * \
                        (center_to_front / wheelbase) / (CivicParams.CENTER_TO_FRONT / CivicParams.WHEELBASE)

  return tire_stiffness_front, tire_stiffness_rear


def dbc_dict(pt_dbc, radar_dbc, chassis_dbc=None, body_dbc=None):
  return {'pt': pt_dbc, 'radar': radar_dbc, 'chassis': chassis_dbc, 'body': body_dbc}

def wiggle(apply_steer,apply_steer_last):

  if apply_steer == apply_steer_last:
    apply_steer +=1

  return int(round(apply_steer))
#alternate settings when using torque interceptor. May or may not be useful to some users/branches.
def apply_ti_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):
  # limits due to driver torque
  driver_max_torque = LIMITS.TI_STEER_MAX + (LIMITS.TI_STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.TI_STEER_DRIVER_FACTOR) * LIMITS.TI_STEER_DRIVER_MULTIPLIER
  driver_min_torque = -LIMITS.TI_STEER_MAX + (-LIMITS.TI_STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.TI_STEER_DRIVER_FACTOR) * LIMITS.TI_STEER_DRIVER_MULTIPLIER
  max_steer_allowed = max(min(LIMITS.TI_STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.TI_STEER_MAX, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    if apply_torque > LIMITS.TI_HIGH_BP:
      apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.TI_STEER_DELTA_DOWN_LOW, -LIMITS.TI_STEER_DELTA_UP_LOW),
                          apply_torque_last + LIMITS.TI_STEER_DELTA_UP_LOW)
    else: 
      apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.TI_STEER_DELTA_DOWN, -LIMITS.TI_STEER_DELTA_UP),
                          apply_torque_last + LIMITS.TI_STEER_DELTA_UP)
  else:
    if apply_torque < -(LIMITS.TI_HIGH_BP):
      apply_torque = clip(apply_torque, apply_torque_last - LIMITS.TI_STEER_DELTA_UP_LOW,
                        min(apply_torque_last + LIMITS.TI_STEER_DELTA_DOWN_LOW, LIMITS.TI_STEER_DELTA_UP_LOW))
    else:
      apply_torque = clip(apply_torque, apply_torque_last - LIMITS.TI_STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.TI_STEER_DELTA_DOWN, LIMITS.TI_STEER_DELTA_UP))

  return int(round(float(apply_torque)))

def apply_std_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):

  # limits due to driver torque
  driver_max_torque = LIMITS.STEER_MAX + (LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
  driver_min_torque = -LIMITS.STEER_MAX + (-LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
  max_steer_allowed = max(min(LIMITS.STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.STEER_MAX, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

def apply_serial_steering_torque_mod(apply_steer, torque_boost_min, steer_warning_counter, steer_cooldown_counter):
  # Init Local Variables
  TORQUE_OVERCLOCK = 238
  TORQUE_STEERING_CAP = 238
  TORQUE_WARNING_COUNTER = 4
  TORQUE_COOLDOWN = 2
  TORQUE_MULTIPLIER = 1

  # Start with old steer copy
  new_steer = apply_steer

  # Apply correct formula based on postive/negative apply_steer
  if new_steer > 0: 
    TORQUE_MULTIPLIER = (1 + (0.1 - (((apply_steer-torque_boost_min)/(TORQUE_STEERING_CAP-torque_boost_min)) / 10)) + 0.04)
    new_steer = min(int(round(new_steer * TORQUE_MULTIPLIER)), TORQUE_STEERING_CAP)
  else:
    TORQUE_MULTIPLIER = (1 + (0.1 - (((apply_steer+torque_boost_min)/(-TORQUE_STEERING_CAP+torque_boost_min)) / 10)) + 0.04)
    new_steer = max(int(round(new_steer * TORQUE_MULTIPLIER)), -TORQUE_STEERING_CAP)
  # Reset the steering torque when the warning counter is too high
  if (new_steer > TORQUE_OVERCLOCK) or (new_steer < -TORQUE_OVERCLOCK):
    steer_warning_counter += 1
    if (steer_warning_counter >= TORQUE_WARNING_COUNTER):
      # apply torque limits steering backup before EPS error & cooldown
      new_steer = apply_steer
      steer_cooldown_counter += 1
      # reset the torque warning after cooldown is done 
      if steer_cooldown_counter >= TORQUE_COOLDOWN:
        steer_warning_counter = 0
        steer_cooldown_counter = 0
  else:
    # Normal torque range (near warning)
    steer_warning_counter = 0
    steer_cooldown_counter = 0


  return new_steer

def apply_toyota_steer_torque_limits(apply_torque, apply_torque_last, motor_torque, LIMITS):
  # limits due to comparison of commanded torque VS motor reported torque
  max_lim = min(max(motor_torque + LIMITS.STEER_ERROR_MAX, LIMITS.STEER_ERROR_MAX), LIMITS.STEER_MAX)
  min_lim = max(min(motor_torque - LIMITS.STEER_ERROR_MAX, -LIMITS.STEER_ERROR_MAX), -LIMITS.STEER_MAX)

  apply_torque = clip(apply_torque, min_lim, max_lim)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque,
                        max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque,
                        apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))


def crc8_pedal(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


def create_gas_interceptor_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  dat = packer.make_can_msg("GAS_COMMAND", 0, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 0, values)


def make_can_msg(addr, dat, bus):
  return [addr, 0, dat, bus]


def get_safety_config(safety_model, safety_param = None):
  ret = car.CarParams.SafetyConfig.new_message()
  ret.safetyModel = safety_model
  if safety_param is not None:
    ret.safetyParam = safety_param
  return ret
