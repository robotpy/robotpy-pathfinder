#!/usr/bin/env python3

import pathfinder as pf
import math

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

    # Wheelbase Width = 0.5m
    modifier = pf.modifiers.TankModifier(trajectory).modify(0.5)

    # Do something with the new Trajectories...
    left = modifier.getLeftTrajectory()
    right = modifier.getRightTrajectory()
