from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import DBC, LKAS_LIMITS

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False
    self.cts_active = False
    self.mrcc_button = 0
    self.cts_button = 0
    self.prev_mrcc_button = 0
    self.velocity_control_mode = False
    self.prev_cts_active = False
    self.prev_cruise_enabled = False
    self.cruise_speed_target_kph = 0.0

    self.distance_button = 0
    self.accel_button = 0
    self.decel_button = 0
    self.set_plus_button = 0
    self.set_minus_button = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # GEAR_BOX: 0=P, 14=R, 1..6=current AT gear, 15=shifting (per DBC comment)
    # DEBUG: pass raw value straight through
    ret.gearStep = int(cp.vl["GEAR"]["GEAR_BOX"])

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    # TODO: this should be from 0 - 1.
    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]

    # True whenever the brake lamp is on — including CTS/MRCC automatic braking.
    # PEDALS.BRAKE_ON and BRAKE.BRAKE_PRESSURE are driver-pedal only; TRACTION.BRAKE
    # is the vehicle-level "brakes are being applied" flag from the ABS/DSC module.
    ret.brakeLamp = cp.vl["TRACTION"]["BRAKE"] == 1

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
    #       it should be used for carState.cruiseState.nonAdaptive instead
    ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
    ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # CTS mode indicator: MSG_10 (0x4f3) byte[5] bit 4. Active in CTS mode only, not MRCC.
    self.cts_active = cp.vl["MSG_10"]["CTS_ACTIVE"] == 1

    # stock lkas should be on
    # TODO: is this needed?
    ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation.
    # Ignore LKAS_BLOCK below 15kph as the EPS normally blocks LKAS at low speed.
    ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked and speed_kph > 15

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]
    self.mrcc_button = cp.vl["CRZ_BTNS"]["MRCC_BUTTON"]
    self.cts_button = cp.vl["CRZ_BTNS"]["CTS_BUTTON"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1

    # cruise control button events: distance, inc, and dec
    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    prev_set_plus_button = self.set_plus_button
    prev_set_minus_button = self.set_minus_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]
    self.accel_button = cp.vl["CRZ_BTNS"]["RES"]
    self.decel_button = cp.vl["CRZ_BTNS"]["SET_M"]
    self.set_plus_button = cp.vl["CRZ_BTNS"]["SET_P"]
    self.set_minus_button = cp.vl["CRZ_BTNS"]["SET_M"]

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
    ]

    # velocity_control_mode: MRCC button rising edge arms it; CTS-mode exit clears it.
    # cts_active is the CTS_ACTIVE bit on MSG_10 — it stays True across short brake
    # interventions that drop cruiseState.enabled, so the driver can resume without
    # losing the upper bound. The mode clears only when the driver actually leaves
    # CTS mode (e.g. by pressing CTS to switch back to MRCC, or by turning off ACC).
    #
    # While the mode is set, cruise_speed_target_kph overrides cruiseState.speed so
    # plannerd's MPC upper bound is decoupled from CRZ_SPEED (breaks the SET_P
    # feedback loop that previously ran away — see longtitude-control-mazda6.md).
    # speedCluster is pinned to the real CRZ_SPEED so the HUD/cluster reading stays
    # honest.
    if self.mrcc_button == 1 and self.prev_mrcc_button == 0:
      self.velocity_control_mode = True
    if self.prev_cts_active and not self.cts_active:
      self.velocity_control_mode = False
    # Force-clear when ACC itself becomes unavailable — also resets the target so
    # next engage cannot reuse a stale value, and prevents missing the cts_active
    # falling edge (e.g. CTS pressed after ACC off).
    if not ret.cruiseState.available:
      self.velocity_control_mode = False
      self.cruise_speed_target_kph = 0.0
    # Initialize target on cruise-enabled rising edge while armed *and* target
    # is still unset (== 0). CTS_ACTIVE is latched by the car across ignition
    # cycles when openpilot doesn't restart, so a CTS-rising-edge initializer
    # can be missed at run start. cruiseState.enabled rising edge is reliable
    # for first-engage. The "target == 0" guard prevents a brake-intervention
    # disengage/resume from clobbering the held target.
    if (self.velocity_control_mode and ret.cruiseState.enabled
        and not self.prev_cruise_enabled and self.cruise_speed_target_kph == 0.0):
      self.cruise_speed_target_kph = ret.cruiseState.speed * CV.MS_TO_KPH
    self.prev_mrcc_button = self.mrcc_button
    self.prev_cts_active = self.cts_active
    self.prev_cruise_enabled = ret.cruiseState.enabled

    # Adjust target on SET+/SET- rising edges (±5 km/h, clamped 30..120).
    # Note: self.accel_button reads the RES (resume) bit, not SET_P — SET_P has
    # its own bit in CRZ_BTNS. Use the dedicated set_plus/minus edges here.
    # Only respond while cruise is engaged so a press during a brake-induced
    # disengage doesn't shift the upper bound silently.
    if self.velocity_control_mode and ret.cruiseState.enabled:
      if self.set_plus_button == 1 and prev_set_plus_button == 0:
        self.cruise_speed_target_kph = min(120.0, self.cruise_speed_target_kph + 5.0)
      if self.set_minus_button == 1 and prev_set_minus_button == 0:
        self.cruise_speed_target_kph = max(30.0, self.cruise_speed_target_kph - 5.0)
    # Override speed whenever the mode is held, so a brake intervention that drops
    # cruiseState.enabled briefly doesn't reset plannerd's upper bound to whatever
    # CRZ_SPEED happens to be at resume time.
    if self.velocity_control_mode:
      # Pin speedCluster to the real CRZ_SPEED *before* overriding speed, otherwise
      # CarInterfaceBase fills speedCluster=speed (= our target) when it's still 0.
      ret.cruiseState.speedCluster = ret.cruiseState.speed
      ret.cruiseState.speed = self.cruise_speed_target_kph * CV.KPH_TO_MS
    ret.mazdaVelocityControlMode = self.velocity_control_mode

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
