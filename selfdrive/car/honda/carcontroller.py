from collections import namedtuple
from cereal import car
from common.realtime import DT_CTRL
from selfdrive.controls.lib.drive_helpers import rate_limit
from common.numpy_fast import clip, interp
from selfdrive.car import create_gas_command, apply_std_steer_torque_limits, apply_serial_steering_torque_mod, apply_ti_steer_torque_limits, wiggle
from selfdrive.car.honda import hondacan
from selfdrive.car.honda.values import CruiseButtons, CAR, VISUAL_HUD, HONDA_BOSCH, HONDA_NIDEC_ALT_PCM_ACCEL, CarControllerParams, SERIAL_STEERING
from opendbc.can.packer import CANPacker
from common.params import Params

VisualAlert = car.CarControl.HUDControl.VisualAlert


def compute_gb_honda_bosch(accel, speed):
  #TODO returns 0s, is unused
  return 0.0, 0.0


def compute_gb_honda_nidec(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake
  return clip(gb, 0.0, 1.0), clip(-gb, 0.0, 1.0)


def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  else:
    return compute_gb_honda_nidec(accel, speed)


#TODO not clear this does anything useful
def actuator_hystereses(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02     # to activate brakes exceed this value
  brake_hyst_off = 0.005   # to deactivate brakes below this value
  brake_hyst_gap = 0.01    # don't change brake command for small oscillations within this value

  #*** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  if (car_fingerprint in (CAR.ACURA_ILX, CAR.CRV, CAR.CRV_EU)) and brake > 0.0:
    brake += 0.15

  return brake, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_on_state, ts):
  if (apply_brake > apply_brake_last):
    pump_on = True

  if (apply_brake == apply_brake_last):
    pump_on = last_pump_on_state

  if (apply_brake < apply_brake_last):
    pump_on = False

  last_pump_on_state = pump_on

  return pump_on, last_pump_on_state


def process_hud_alert(hud_alert):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0

  # priority is: FCW, steer required, all others
  if hud_alert == VisualAlert.fcw:
    fcw_display = VISUAL_HUD[hud_alert.raw]
  elif hud_alert in [VisualAlert.steerRequired, VisualAlert.ldw]:
    steer_required = VISUAL_HUD[hud_alert.raw]
  else:
    acc_alert = VISUAL_HUD[hud_alert.raw]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise",  "car",
                     "lanes", "fcw", "acc_alert", "steer_required", "dashed_lanes", "dist_lines"])


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.signal_last = 0.
    self.apply_brake_last = 0
    self.last_pump_on_state = False
    self.apply_steer_last = 0
    self.apply_steer_last_ti = 0
    self.apply_steer_warning_counter = 0
    self.apply_steer_cooldown_counter = 0
    self.steer_torque_boost_min = 70
    self.packer = CANPacker(dbc_name)

    self.params = CarControllerParams(CP)

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    if enabled:
      accel = actuators.accel
      gas, brake = compute_gas_brake(actuators.accel, CS.out.vEgo, CS.CP.carFingerprint)
    else:
      accel = 0.0
      gas, brake = 0.0, 0.0

    # *** apply brake hysteresis ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hystereses(brake, self.braking, self.brake_steady, CS.out.vEgo, CS.CP.carFingerprint)

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(pre_limit_brake, self.brake_last, -2., DT_CTRL)

    if enabled and CS.out.cruiseState.enabled:
      if hud_show_car:
        hud_car = 2
      else:
        hud_car = 1
    else:
      hud_car = 0

    fcw_display, steer_required, acc_alert = process_hud_alert(hud_alert)

    cur_time = frame * DT_CTRL
    if (CS.leftBlinkerOn or CS.rightBlinkerOn):
      self.signal_last = cur_time

    lkas_active = enabled and not CS.steer_not_allowed and CS.lkasEnabled and ((CS.automaticLaneChange and not CS.belowLaneChangeSpeed) or ((not ((cur_time - self.signal_last) < 1) or not CS.belowLaneChangeSpeed) and not (CS.leftBlinkerOn or CS.rightBlinkerOn)))

    # **** process the car messages ****

    if CS.CP.enableTorqueInterceptor:
      new_steer = int(round(actuators.steer * P.TI_STEER_MAX))
      apply_steer_ti = apply_ti_steer_torque_limits(new_steer, self.apply_steer_last_ti,
                                                  CS.out.steeringTorque, P)
      apply_steer_ti = wiggle(apply_steer_ti, self.apply_steer_last_ti)
      self.apply_steer_last_ti = apply_steer_ti

    
    # steer torque is converted back to CAN reference (positive when steering right)
    apply_steer = int(interp(actuators.steer * P.STEER_MAX, P.STEER_LOOKUP_BP, P.STEER_LOOKUP_V))
    if (CS.CP.carFingerprint in SERIAL_STEERING): # Dynamic torque boost if above threshold, smooth torque blend otherwise
      if (apply_steer >= self.steer_torque_boost_min) or (apply_steer <= -self.steer_torque_boost_min):
        apply_steer = apply_serial_steering_torque_mod(apply_steer, self.steer_torque_boost_min, self.apply_steer_warning_counter, self.apply_steer_cooldown_counter)
      else:
        apply_steer = apply_std_steer_torque_limits(apply_steer, -(self.apply_steer_last), CS.out.steeringTorque, self.params)
        self.apply_steer_warning_counter = 0
        self.apply_steer_cooldown_counter = 0
  
    # steer torque is converted back to CAN reference (positive when steering right)
    apply_steer = -apply_steer

    self.apply_steer_last = apply_steer

    # Send CAN commands.
    can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    if CS.CP.carFingerprint in HONDA_BOSCH and CS.CP.openpilotLongitudinalControl:
      if (frame % 10) == 0:
        can_sends.append((0x18DAB0F1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 1))

    
    idx = frame % 4

    #if ti is enabled we don't have to send apply steer to the stock system but a signal should still be sent.
    if CS.CP.enableTorqueInterceptor:
      can_sends.append(hondacan.create_ti_steering_control(self.packer, CS.CP.carFingerprint,apply_steer_ti))
      can_sends.append(hondacan.create_steering_control(self.packer, apply_steer,
        lkas_active, CS.CP.carFingerprint, idx, CS.CP.openpilotLongitudinalControl))
    else:
      #The ti cannot be detected unless OP sends a can message to it becasue the ti only transmits when it 
      #sees the signature key in the designated address range.
      can_sends.append(hondacan.create_steering_control(self.packer, apply_steer,
      lkas_active, CS.CP.carFingerprint, idx, CS.CP.openpilotLongitudinalControl))
      apply_steer_ti = 0
      can_sends.append(hondacan.create_ti_steering_control(self.packer, CS.CP.carFingerprint, apply_steer_ti))


    # TODO: pass in LoC.long_control_state and use that to decide starting/stoppping
    stopping = accel < 0 and CS.out.vEgo < P.STOPPING_SPEED
    starting = accel > 0 and CS.out.vEgo < P.STARTING_SPEED

    # Prevent rolling backwards
    accel = -4.0 if stopping else accel

    # wind brake from air resistance decel at high speed
    wind_brake = interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15])
    # all of this is only relevant for HONDA NIDEC
    max_accel = interp(CS.out.vEgo, P.NIDEC_MAX_ACCEL_BP, P.NIDEC_MAX_ACCEL_V)
    # TODO this 1.44 is just to maintain previous behavior
    pcm_speed_BP = [-wind_brake,
                    -wind_brake*(3/4),
                      0.0,
                      0.5]
    # The Honda ODYSSEY seems to have different PCM_ACCEL
    # msgs, is it other cars too?
    if CS.CP.enableGasInterceptor:
      pcm_speed = 0.0
      pcm_accel = int(0.0)
    elif CS.CP.carFingerprint in HONDA_NIDEC_ALT_PCM_ACCEL:
      pcm_speed_V = [0.0,
                     clip(CS.out.vEgo - 3.0, 0.0, 100.0),
                     clip(CS.out.vEgo + 0.0, 0.0, 100.0),
                     clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = interp(gas-brake, pcm_speed_BP, pcm_speed_V)
      pcm_accel = int((1.0) * 0xc6)
    else:
      pcm_speed_V = [0.0,
                     clip(CS.out.vEgo - 2.0, 0.0, 100.0),
                     clip(CS.out.vEgo + 2.0, 0.0, 100.0),
                     clip(CS.out.vEgo + 3.0, 0.0, 100.0)]
      pcm_speed = interp(gas-brake, pcm_speed_BP, pcm_speed_V)
      pcm_accel = int(clip((accel/1.44)/max_accel, 0.0, 1.0) * 0xc6)


    if not CS.CP.openpilotLongitudinalControl:
      if (frame % 2) == 0:
        idx = frame // 2
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, CS.CP.carFingerprint, idx))
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.CANCEL, idx, CS.CP.carFingerprint))
      elif CS.out.cruiseState.standstill:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, idx, CS.CP.carFingerprint))

    else:
      # Send gas and brake commands.
      if (frame % 2) == 0:
        idx = frame // 2
        ts = frame * DT_CTRL

        if CS.CP.carFingerprint in HONDA_BOSCH:
          if CS.out.cruiseState.enabled:
            bosch_gas = interp(accel, P.BOSCH_GAS_LOOKUP_BP, P.BOSCH_GAS_LOOKUP_V)
            can_sends.extend(hondacan.create_acc_commands(self.packer, enabled, accel, bosch_gas, idx, stopping, starting, CS.CP.carFingerprint))

        else:
          apply_brake = clip(self.brake_last - wind_brake, 0.0, 1.0)
          apply_brake = int(clip(apply_brake * P.BRAKE_MAX, 0, P.BRAKE_MAX - 1))
          if not CS.out.cruiseState.enabled and not (CS.CP.pcmCruise and CS.accEnabled and CS.CP.minEnableSpeed > 0 and not CS.out.cruiseState.enabled):
            apply_brake = 0.
          pump_on, self.last_pump_on_state = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_on_state, ts)
          # Do NOT send the cancel command if we are using the pedal. Sending cancel causes the car firmware to
          # turn the brake pump off, and we don't want that. Stock ACC does not send the cancel cmd when it is braking.

        if CS.CP.enableGasInterceptor:
          pcm_cancel_cmd = False

          pcm_override = True
          can_sends.append(hondacan.create_brake_command(self.packer, apply_brake, pump_on,
            pcm_override, pcm_cancel_cmd, fcw_display, idx, CS.CP.carFingerprint, CS.stock_brake))
          self.apply_brake_last = apply_brake

          if CS.CP.enableGasInterceptor:
            # way too aggressive at low speed without this
            gas_mult = interp(CS.out.vEgo, [0., 10.], [0.4, 1.0])
            # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
            # This prevents unexpected pedal range rescaling
            apply_gas = clip(gas_mult * (gas - brake + wind_brake*3/4), 0., 1.)
            if not CS.out.cruiseState.enabled:
              apply_gas = 0.
            can_sends.append(create_gas_command(self.packer, apply_gas, idx))

    hud = HUDData(int(pcm_accel), (int(round(hud_v_cruise)) if hud_car != 0 else 255), hud_car,
                  hud_show_lanes and lkas_active, fcw_display, acc_alert, steer_required, CS.lkasEnabled and not lkas_active, CS.read_distance_lines)

    # Send dashboard UI commands.
    if (frame % 10) == 0:
      idx = (frame//10) % 4
      can_sends.extend(hondacan.create_ui_commands(self.packer, pcm_speed, hud, enabled, stopping, CS.CP.carFingerprint, CS.is_metric, idx, CS.CP.openpilotLongitudinalControl, CS.stock_hud))

    return can_sends
