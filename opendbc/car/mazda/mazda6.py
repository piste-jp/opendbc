import math

from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.values import Buttons

# The EPS sets LKAS_BLOCK at low speed during normal operation, don't report it as a fault there
STEER_FAULT_MIN_SPEED_KPH = 15.

# Velocity control mode: SET+/SET- move the driver's upper bound for plannerd in fixed steps,
# and openpilot presses SET+/SET- itself so the stock ACC set speed (CRZ_SPEED) follows the plan.
VC_SPEED_STEP_KPH = 5.
VC_SPEED_MIN_KPH = 30.
VC_SPEED_MAX_KPH = 120.
# Keep CRZ_SPEED a few km/h above the plan target so the stock ACC can still accelerate when a lead pulls away
VC_RAISE_GAP_KPH = 2.
VC_LOWER_GAP_KPH = 7.
# CRZ_SPEED needs time to settle after a press, sending faster makes presses pile up
VC_BUTTON_COOLDOWN_FRAMES = 100  # 1 s


class Mazda6CarState(CarState):
  def __init__(self, CP):
    super().__init__(CP)

    self.set_plus_button = 0
    self.set_minus_button = 0
    self.prev_cruise_enabled = False

    self.cts_mode = False
    self.velocity_control_mode = False
    self.cruise_speed_target_kph = 0.

  def update(self, can_parsers) -> structs.CarState:
    ret = super().update(can_parsers)
    cp = can_parsers[Bus.pt]

    mazda = ret.mazda

    # GEAR_BOX: 0=P, 1..6=current AT gear, 14=R, 15=shifting
    gear_box = int(cp.vl["GEAR"]["GEAR_BOX"])
    mazda.gearStep = gear_box if 1 <= gear_box <= 6 else -1
    mazda.manualShiftMode = cp.vl["GEAR"]["MANUAL_MODE"] == 1
    mazda.shifterM = cp.vl["NEW_MSG_6"]["SHIFTER_M"] == 1

    # PEDALS.BRAKE_ON is the driver's pedal only, TRACTION.BRAKE also covers braking by the stock ACC
    mazda.brakeLamp = cp.vl["TRACTION"]["BRAKE"] == 1

    # telemetry, for logs
    mazda.engineRpm = cp.vl["ENGINE_DATA"]["RPM"]
    mazda.gasPedalRaw = cp.vl["ENGINE_DATA"]["PEDAL_GAS"]
    mazda.accelX = cp.vl["BRAKE"]["VEHICLE_ACC_X"]
    mazda.accelY = cp.vl["BRAKE"]["VEHICLE_ACC_Y"]
    mazda.brakePressure = cp.vl["BRAKE"]["BRAKE_PRESSURE"]
    mazda.coolantTemp = cp.vl["CHECK_AND_TEMP"]["COOLANT_TEMP"]

    ret.steerFaultTemporary = ret.steerFaultTemporary and cp.vl["ENGINE_DATA"]["SPEED"] > STEER_FAULT_MIN_SPEED_KPH

    # In CTS mode the stock ACC and camera drive the car (the panda forwards the camera's LKAS commands).
    # openpilot stays out: report cruise as unavailable so it never engages or cancels, and hide faults.
    self.cts_mode = cp.vl["CRZ_INFO"]["CTS_MODE"] == 1
    mazda.ctsMode = self.cts_mode
    if self.cts_mode:
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False
      ret.steerFaultTemporary = False
      ret.steerFaultPermanent = False
      ret.invalidLkasSetting = False
      ret.lowSpeedAlert = False

    self.update_velocity_control(ret, cp)
    return ret

  def update_velocity_control(self, ret: structs.CarState, cp) -> None:
    prev_set_plus_button = self.set_plus_button
    prev_set_minus_button = self.set_minus_button
    self.set_plus_button = cp.vl["CRZ_BTNS"]["SET_P"]
    self.set_minus_button = cp.vl["CRZ_BTNS"]["SET_M"]

    cruise = ret.cruiseState

    # The mode is on while the stock ACC is on in MRCC mode, and the next ACC on starts from a fresh target
    self.velocity_control_mode = cruise.available
    if not cruise.available:
      self.cruise_speed_target_kph = 0.

    # Initialize the target on engage, rounded up onto the SET+/SET- grid. A brake intervention
    # disengages and re-engages ACC without resetting it.
    if self.velocity_control_mode and cruise.enabled and not self.prev_cruise_enabled and self.cruise_speed_target_kph == 0.:
      target_kph = math.ceil(cruise.speed * CV.MS_TO_KPH / VC_SPEED_STEP_KPH) * VC_SPEED_STEP_KPH
      self.cruise_speed_target_kph = min(max(target_kph, VC_SPEED_MIN_KPH), VC_SPEED_MAX_KPH)

    # Only move the target while engaged, so a press during a brake intervention doesn't shift it silently
    if self.velocity_control_mode and cruise.enabled:
      if self.set_plus_button == 1 and prev_set_plus_button == 0:
        self.cruise_speed_target_kph = min(self.cruise_speed_target_kph + VC_SPEED_STEP_KPH, VC_SPEED_MAX_KPH)
      if self.set_minus_button == 1 and prev_set_minus_button == 0:
        self.cruise_speed_target_kph = max(self.cruise_speed_target_kph - VC_SPEED_STEP_KPH, VC_SPEED_MIN_KPH)

    self.prev_cruise_enabled = cruise.enabled

    # While the mode is on, the target replaces CRZ_SPEED as plannerd's upper bound, so it doesn't
    # feed back on the SET+/SET- presses the carcontroller makes. The cluster keeps the real CRZ_SPEED.
    if self.velocity_control_mode:
      cruise.speedCluster = cruise.speed
      cruise.speed = self.cruise_speed_target_kph * CV.KPH_TO_MS

    ret.mazda.velocityControlMode = self.velocity_control_mode


class Mazda6CarController(CarController):
  FORWARD_TJA = True

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.vc_button_cooldown = 0

  def update(self, CC, CS, now_nanos):
    new_actuators, can_sends = super().update(CC, CS, now_nanos)

    # the stock camera steers in CTS mode, don't send anything
    if CS.cts_mode:
      return new_actuators, []

    if self.vc_button_cooldown > 0:
      self.vc_button_cooldown -= 1

    # Velocity control mode: press SET+/SET- so the stock ACC set speed tracks plannerd's target,
    # capped by the driver's upper bound
    if (CS.velocity_control_mode and CS.out.cruiseState.enabled and self.vc_button_cooldown == 0 and
        not CC.cruiseControl.cancel and not CC.cruiseControl.resume and CC.mazda.planTargetKph > 0):
      target_kph = min(CC.mazda.planTargetKph, CS.cruise_speed_target_kph)
      crz_speed_kph = CS.out.cruiseState.speedCluster * CV.MS_TO_KPH
      gap_kph = crz_speed_kph - target_kph

      button = Buttons.NONE
      if gap_kph <= VC_RAISE_GAP_KPH and crz_speed_kph < CS.cruise_speed_target_kph:
        button = Buttons.SET_PLUS
      elif gap_kph >= VC_LOWER_GAP_KPH:
        button = Buttons.SET_MINUS

      if button != Buttons.NONE:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, button))
        self.vc_button_cooldown = VC_BUTTON_COOLDOWN_FRAMES

    return new_actuators, can_sends
