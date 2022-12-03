from cereal import car
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.mazda import mazdacan
from selfdrive.car.mazda.values import CarControllerParams, Buttons, GEN1, GEN2
from common.realtime import DT_CTRL
from typing import List


VisualAlert = car.CarControl.HUDControl.VisualAlert

class DT:
  """ Used to create long duration timer objects\n
  Example usage: timer_obj = DT(duration=31540000.01) # create a timer object with a duration of 1 year + 10ms\n
  ret = timer_obj.active() # Poll this. Returns true if time since last reset is less than duration \n
  DT.tick(DT_CTRL) # call this function at DT_CTRL frequency (DT_CTRL=0.01s, 100Hz)\n
  timer_obj.reset() # Resets the timer\n
  timer_obj.adjust(10.0) # Adjusts the duration of the timer\n
  timer_obj.once_after_reset() # Returns true only one time after reset\n
  Resets on overflow at float("inf") or float("-inf") or timer_obj.max or min\n"""
  objects = [] # type: List[DT]
  def __init__(self, duration=0) -> None:
    self.duration = duration
    self.was_reset = False
    self.timer = 0
    self.min = float("-inf") # type: float
    self.max = float("inf") # type: float
    self.__class__.objects.append(self)

  @classmethod
  def tick(cls, dt=DT_CTRL) -> None:
    """Call this every frame DT_CTRL=0.01s.\n
    Resets on overflow at float("inf") or float("-inf") """
    for obj in cls.objects:
      obj.timer += dt
      # reset on overflow
      obj.reset() if (obj.timer == (obj.max or obj.min)) else None

  @classmethod
  def reset_all(cls) -> None:
    """Resets timer for all DT objects"""
    for obj in cls.objects:
      obj.reset()

  def reset(self) -> None:
    """Resets this objects timer"""
    self.timer = 0
    self.was_reset = True

  def active(self) -> bool:
    """Returns true if time since last reset is less than duration"""
    return bool(self.timer <= self.duration)

  def adjust(self, duration) -> None:
    """Adjusts the duration of the timer"""
    self.duration = duration

  def once_after_reset(self) -> bool: 
    """Returns true only one time after calling reset()"""
    ret = self.was_reset
    self.was_reset = False
    return ret

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.packer = CANPacker(dbc_name)
    self.brake_counter = 0
    self.frame = 0
    self.params = CarControllerParams(self.CP)
    self.ts_last = 0
    # create long duration timers
    self.hold_timer = DT(6.0)
    self.resume_timer = DT(0.5)

  def update(self, CC, CS):
    can_sends = []

    apply_steer = 0
    
    #update the timer
    DT.tick()

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last,
                                                  CS.out.steeringTorque, self.params)
    self.apply_steer_last = apply_steer

    if CS.CP.carFingerprint in GEN1:
      if CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter, Buttons.CANCEL))
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
          # Send Resume button when planner wants car to move
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP.carFingerprint, CS.crz_btns_counter, Buttons.RESUME))

      # send HUD alerts
      if self.frame % 50 == 0:
        ldw = CC.hudControl.visualAlert == VisualAlert.ldw
        steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
        # TODO: find a way to silence audible warnings so we can add more hud alerts
        steer_required = steer_required and CS.lkas_allowed_speed
        can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))
    
    elif CS.CP.carFingerprint in GEN2:
      resume = False
      hold = False
      if self.frame % 2 == 0:
        if CS.out.standstill and not (CC.cruiseControl.resume or CC.cruiseControl.override):
          hold = self.hold_timer.active()
        #elif CC.cruiseControl.resume:
          #resume = self.resume_timer.active()
        else :
          DT.reset_all()
        resume = (CC.cruiseControl.resume or CC.cruiseControl.override) if CS.out.standstill else False

        can_sends.extend(mazdacan.create_acc_cmd(self, self.packer, CS, CC, hold, resume))
    
    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP.carFingerprint,
                                                      self.frame, apply_steer, CS.cam_lkas))



    new_actuators = CC.actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    self.frame += 1
    return new_actuators, can_sends

