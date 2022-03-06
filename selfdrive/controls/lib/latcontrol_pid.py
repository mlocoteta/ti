import math

from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from cereal import log
from common.op_params import opParams, LAT_PID_KP_BP, LAT_PID_KP_V, LAT_PID_KD_BP, LAT_PID_KD_V, LAT_PID_KI_BP, LAT_PID_KI_V, LAT_PID_KF


class LatControlPID(LatControl):
  def __init__(self, CP, CI, OP=None):
    super().__init__(CP, CI)
    if OP is None:
      OP = opParams()
    self.op_params = OP

    kp = (LAT_PID_KP_BP, LAT_PID_KP_V)
    ki = (LAT_PID_KI_BP, LAT_PID_KI_V)
    kd = (LAT_PID_KD_BP, LAT_PID_KD_V)
    kf = LAT_PID_KF
    self.pid = PIDController(kp, ki, kd, kf, pos_limit=1.0, neg_limit=-1.0,
                            derivative_period=0.1, isLateral=True)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, CP, VM, params, last_actuators, desired_curvature, desired_curvature_rate):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg

    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = angle_steers_des - CS.steeringAngleDeg
    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      steers_max = get_steer_max(CP, CS.vEgo)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max

      # offset does not contribute to resistive torque
      steer_feedforward = self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)

      deadzone = 0.0

      output_steer = self.pid.update(angle_steers_des, CS.steeringAngleDeg, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = self._check_saturation(steers_max - abs(output_steer) < 1e-3, CS)

    return output_steer, angle_steers_des, pid_log