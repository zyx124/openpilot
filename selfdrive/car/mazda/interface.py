#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.mazda.values import CAR, LKAS_LIMITS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive import global_ti as TI
from selfdrive import global_ri as RI

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    print("in get_params, entering get_std_params")
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "mazda"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.mazda)]
    ret.radarOffCan = True

    ret.dashcamOnly = False # candidate not in [CAR.CX9_2021]

    ret.openpilotLongitudinalControl = True
    ret.longitudinalTuning.kpBP = [0., 10.]
    ret.longitudinalTuning.kpV = [.5, 0.3,]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.06]

    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 1.0
    ret.steerLimitTimer = 0.8
    tire_stiffness_factor = 0.70   # not optimized yet

    if ret.enableTorqueInterceptor:
      print("Adjusting PID parameters for TI")
      if candidate == CAR.CX5:
        ret.mass = 3655 * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 2.7
        ret.steerRatio = 15.5
        ret.lateralTuning.pid.kiBP = [5.0, 25.0]
        ret.lateralTuning.pid.kpBP = [5.0, 25.0]
        ret.lateralTuning.pid.kpV = [0.25,0.28]
        ret.lateralTuning.pid.kiV = [0.01,0.025]
        ret.lateralTuning.pid.kf = 0.00008

        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [5.0, 35]
        ret.lateralTuning.indi.innerLoopGainV = [4.5, 6.0]
        ret.lateralTuning.indi.outerLoopGainBP = [5, 35]
        ret.lateralTuning.indi.outerLoopGainV = [3.0, 6]
        ret.lateralTuning.indi.timeConstantBP = [2, 35]
        ret.lateralTuning.indi.timeConstantV = [0.2, 1.5]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0, 25]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.0, 1]

      elif candidate in [CAR.CX9, CAR.CX9_2021]:
        ret.mass = 4217 * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 3.1
        ret.steerRatio = 17.6
        ret.lateralTuning.pid.kiBP = [8.0, 30.0]
        ret.lateralTuning.pid.kpBP = [8.0, 30.0]
        ret.lateralTuning.pid.kpV = [0.10,0.22]
        ret.lateralTuning.pid.kiV = [0.01,0.019]
        ret.lateralTuning.pid.kf = 0.00006
      elif candidate == CAR.MAZDA3:
        ret.mass = 2875 * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 2.7
        ret.steerRatio = 14.0

        ret.lateralTuning.pid.kiBP = [5.0, 25.0]
        ret.lateralTuning.pid.kpBP = [5.0, 25.0]
        ret.lateralTuning.pid.kpV = [0.25,0.28]
        ret.lateralTuning.pid.kiV = [0.01,0.025]
        ret.lateralTuning.pid.kf = 0.00008

        ret.lateralTuning.init('indi')
        ret.lateralTuning.indi.innerLoopGainBP = [5.0, 35]
        ret.lateralTuning.indi.innerLoopGainV = [4.5, 6.0]
        ret.lateralTuning.indi.outerLoopGainBP = [5, 35]
        ret.lateralTuning.indi.outerLoopGainV = [3.0, 6]
        ret.lateralTuning.indi.timeConstantBP = [2, 35]
        ret.lateralTuning.indi.timeConstantV = [0.2, 1.5]
        ret.lateralTuning.indi.actuatorEffectivenessBP = [0, 25]
        ret.lateralTuning.indi.actuatorEffectivenessV = [2.0, 1]
      elif candidate == CAR.MAZDA6:
        ret.mass = 3443 * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 2.83
        ret.steerRatio = 15.5
        ret.lateralTuning.pid.kiBP = [8.0, 30.0]
        ret.lateralTuning.pid.kpBP = [8.0, 30.0]
        ret.lateralTuning.pid.kpV = [0.10,0.22]
        ret.lateralTuning.pid.kiV = [0.01,0.019]
        ret.lateralTuning.pid.kf = 0.00006
    else:
      if candidate == CAR.CX5:
        ret.mass = 3655 * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 2.7
        ret.steerRatio = 15.5
        ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.19], [0.019]]
        ret.lateralTuning.pid.kf = 0.00006
      elif candidate in [CAR.CX9, CAR.CX9_2021]:
        ret.mass = 4217 * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 3.1
        ret.steerRatio = 17.6
        ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.19], [0.019]]
        ret.lateralTuning.pid.kf = 0.00006
      elif candidate == CAR.MAZDA3:
        ret.mass = 2875 * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 2.7
        ret.steerRatio = 14.0
        ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.19], [0.019]]
        ret.lateralTuning.pid.kf = 0.00006
      elif candidate == CAR.MAZDA6:
        ret.mass = 3443 * CV.LB_TO_KG + STD_CARGO_KG
        ret.wheelbase = 2.83
        ret.steerRatio = 15.5
        ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.19], [0.019]]
        ret.lateralTuning.pid.kf = 0.00006

    # No steer below disable speed
    ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.centerToFront = ret.wheelbase * 0.41

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):

    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)
    if self.CP.enableTorqueInterceptor and not TI.enabled:
      TI.enabled = True
      self.cp = self.CS.get_can_parser(self.CP)
    if self.CP.radarInterceptMode and not RI.enabled:
      RI.enabled = True
      self.cp = self.CS.get_can_parser(self.CP)
      self.cp_cam = self.CS.get_cam_can_parser(self.CP)
    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    # events
    events = self.create_common_events(ret)

    if not self.CS.acc_active_last and not self.CS.ti_lkas_allowed:
      events.add(EventName.steerTempUnavailable)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    can_sends = self.CC.update(c, self.CS, self.frame)
    self.frame += 1
    return can_sends
