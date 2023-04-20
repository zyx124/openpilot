#!/usr/bin/env python3

""" gets behavior input from UI and saves it to params"""
from common.params import Params
import cereal.messaging as messaging
import time

A_CRUISE_MIN = -1.2
class LiveBehavior():
  def __init__(self):
    self.a_cruise_min = A_CRUISE_MIN
    self.a_cruise_max_factor = 1.0
    self.sm = messaging.SubMaster(['behavior'])
    
  def update(self):
    self.sm.update(0)
    if self.sm.updated['behavior']:
      acmin = self.sm['behavior'].accelCruiseMin
      acmaxf = self.sm['behavior'].accelCruiseMaxFactor
      self.a_cruise_min = acmin if acmin >= -3.0 and acmin <= 0.0 else A_CRUISE_MIN
      self.a_cruise_max_factor = acmaxf if acmaxf >= 0.0 and acmaxf <= 3.0 else 1.0
      
      
  
  def get_live_a_cruise_min(self):
    self.update()
    print("a_cruise_min: ", self.a_cruise_min)
    return self.a_cruise_min
  
  def get_live_a_cruise_max_factor(self):
    self.update()
    print("a_cruise_max_factor: ", self.a_cruise_max_factor)
    return self.a_cruise_max_factor
  
  

lb = LiveBehavior()

class Behaviord:
  def __init__(self):
    
    sm = messaging.SubMaster(['behavior'])
    self.sm = sm
    self.p = Params()
    self.init_params()

  
  def init_params(self):
    if self.p.get("AccelCruiseMin") is None:
      self.p.put("AccelCruiseMin", str(A_CRUISE_MIN))
    if self.p.get("AccelCruiseMaxFactor") is None:
      self.p.put("AccelCruiseMaxFactor", "1.0")

  def save(self):
    if self.sm.updated['behavior']:
      print("behavior updated")
      self.p.put("AccelCruiseMin", str(self.sm['behavior'].accelCruiseMin))
      self.p.put("AccelCruiseMaxFactor", str(self.sm['behavior'].accelCruiseMaxFactor))

  def behaviord_thread(self):
    while 1:
      self.sm.update(0)
      self.save()
      #self.send()
      time.sleep(1)

def main():
  behaviord = Behaviord()
  behaviord.behaviord_thread()
            
if __name__ == "__main__":
  main()
