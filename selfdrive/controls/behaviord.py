#!/usr/bin/env python3

""" gets behavior input from UI and saves it to params"""
from common.params import Params
from cereal import log
from cereal.messaging import SubMaster
import time

sm = SubMaster(['behavior'])
p = Params()

def init_params():
  if p.get("ComfortBrake") is None:
    p.put("ComfortBrake", "0")
  # Add more params here
        
def save():
  if sm.updated['behavior']:
    p.put("ComfortBrake", str(sm['behavior'].comfortBrake))
    # Add more params here
    
def behavior_thread():
  init_params()
  while 1:
    sm.update(0)
    save()
    time.sleep(1)
            
            
if __name__ == "__main__":
  behavior_thread()
            