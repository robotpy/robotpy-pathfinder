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
    
    # Wheelbase Width = 0.5m, Wheelbase Depth = 0.6m, Swerve Mode = Default
    modifier = pf.modifiers.SwerveModifier(trajectory).modify(0.5, 0.6)

    # Do something with the new Trajectories...
    fl = modifier.getFrontLeftTrajectory()
    fr = modifier.getFrontRightTrajectory()
    bl = modifier.getBackLeftTrajectory()
    br = modifier.getBackRightTrajectory()
