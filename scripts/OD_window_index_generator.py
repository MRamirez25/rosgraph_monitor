#!/usr/bin/env python
from math import sqrt, pi, sin, cos
import numpy as np
import pickle

ODw_center = [0.5,0.0]
ODw_width = 1.0

# Pars costmap (Hardcoded for now)
costmap_width = 5.0
costmap_res = 0.01
costmap_width_n = int((costmap_width/costmap_res))

yaws = np.arange(-pi,pi,0.1)
# print(yaws)
ODw_n = dict()
for yaw in yaws:
    ODw_n[yaw] = []
    for n in range(costmap_width_n**2):
        x = (n%costmap_width_n)*costmap_res - costmap_width/2
        y = int(n/costmap_width_n)*costmap_res - costmap_width/2
        xa = x*cos(yaw)+y*sin(yaw)
        ya = -x*sin(yaw)+y*cos(yaw)
        if xa < ODw_center[0]+(ODw_width/2) and xa > ODw_center[0]+(-ODw_width/2) and ya < ODw_center[1]+(ODw_width/2) and ya > ODw_center[1]-(ODw_width/2):
            ODw_n[yaw].append(n)

filename = 'OD_windows'
with open(filename, 'wb') as file:
    pickle.dump([ODw_n,yaws], file)

