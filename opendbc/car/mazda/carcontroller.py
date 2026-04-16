from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, Buttons

VisualAlert = structs.CarControl.HUDControl.VisualAlert

# Frames to wait between the driver's MRCC press and the CTS press we inject.
# Pressing them too close together kept the stock camera in MRCC; ~2s matches
# how a person physically presses one then the other (100 Hz update rate).
CTS_INJECT_DELAY_FRAMES = 200


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.prev_mrcc_button = 0
    # Countdown until we inject the CTS button after a driver MRCC press.
    # 0 means no pending injection. Positive values count down each frame.
    self.cts_inject_countdown = 0
    self.cts_inject_pulses_left = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # When the driver presses MRCC, schedule a CTS press ~2s later so the
    # stock camera switches to CTS (openpilot lateral control works in CTS,
    # but causes a stock sensor error in MRCC). After that the driver has
    # the same experience either way: the stock CTS handles speed and
    # following, openpilot adds lateral. No openpilot-driven set-speed
    # adjustment — set speed is whatever the stock system shows.
    mrcc_pressed = CS.mrcc_button == 1 and self.prev_mrcc_button == 0
    if mrcc_pressed:
      self.cts_inject_countdown = CTS_INJECT_DELAY_FRAMES
    self.prev_mrcc_button = CS.mrcc_button

    if self.cts_inject_countdown > 0:
      self.cts_inject_countdown -= 1
      if self.cts_inject_countdown == 0:
        self.cts_inject_pulses_left = 5

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams)

    if CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

      elif self.cts_inject_pulses_left > 0 and self.frame % 2 == 0:
        # Inject CTS button press scheduled by the MRCC redirect.
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CTS))
        self.cts_inject_pulses_left -= 1

    self.apply_torque_last = apply_torque

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
