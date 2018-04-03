#!/usr/bin/env python3
#
# Demonstrates plotting a trajectory and waypoints with matplotlib. Obviously,
# you can plot a lot more than that -- use your imagination!
#
# Note: This example requires you to install matplotlib
#

import pathfinder as pf
import math

import matplotlib.pyplot as plt

if __name__ == '__main__':

    points = [
        pf.Waypoint(-4, -1, math.radians(-45.0)),
        pf.Waypoint(-2, -2, 0),
        pf.Waypoint(0, 0, 0),
    ]
    
    info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                                   dt=0.05, # 50ms
                                   max_velocity=1.7,
                                   max_acceleration=2.0,
                                   max_jerk=60.0)
    
    # plot the waypoints
    mx, my = zip(*[(m.y, m.x) for m in points])
    plt.scatter(mx, my, c='r')
    
    # plot the trajectory
    x, y = zip(*[(seg.y, seg.x) for seg in trajectory])
    plt.plot(x, y)
    
    plt.show()
