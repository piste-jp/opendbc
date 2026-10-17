#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.mazda6 import Mazda6CarController, Mazda6CarState
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, MazdaSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP: structs.CarParams):
    if CP.carFingerprint == CAR.MAZDA_6_WAGON_2022:
      self.CarState = Mazda6CarState
      self.CarController = Mazda6CarController
    super().__init__(CP)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = True

    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021, CAR.MAZDA_6_WAGON_2022)

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_6_WAGON_2022):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    if candidate == CAR.MAZDA_6_WAGON_2022:
      # in CTS mode the stock ACC and camera drive the car, openpilot only steers in MRCC mode
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.STOCK_CTS.value
      # lagd learned 0.56 s of lateral delay on this car, and starts from steerActuatorDelay + 0.2 s
      ret.steerActuatorDelay = 0.36

    ret.centerToFront = ret.wheelbase * 0.41

    return ret
