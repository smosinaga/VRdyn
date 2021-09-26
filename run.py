import time
import sys
from pathlib import Path
sys.path.append(Path(__file__))
import solvers.preProcessor as pre


start = time.time()

# tyre, car, bump, sol = pre.read("Car2DOF-1-FRC-Trapezoidal")

tyre, car, bump, sol = pre.read("example2")

end = time.time()
print("Elapsed time: "+str(round(end-start,3))+ " s")
del(start,end)

# model = ["Car2DOF","Car4DOF","Car7DOF",]
# velocity = ["1","5","10"]
# tyres = ["SPC","RRC","FRC"]
# bumps = ["Trapezoidal", "Convex"]

# for i1 in model:
#     for i2 in velocity:
#         for i3 in tyres:
#             for i4 in bumps:
#                 name = i1 + "-" + i2 + "-" + i3 + "-"  + i4 
#                 print(name)

#                 tyre, car, bump, sol = pre.read(name)



"""
PENDIENTES:

Fuerzas estaticas car7DOF

"""
