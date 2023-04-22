from common.params import Params
from selfdrive.controls.behaviord import PARAMS

p = Params()

# delete all params in PARAMS

for param in PARAMS.keys():
    p.remove(param)
    print(f"deleted {param}")