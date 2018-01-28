robotpy-pathfinder
==================

These are python bindings around Jaci R's PathFinder library. From the original
documentation:

Cross-Platform, Multi-Use Motion Profiling and Trajectory Generation.

Pathfinder is a library for generating Motion Profiles, a way to smoothly fit
and follow a trajectory based upon given waypoints.

Note: This requires C++ 11 and Python 3.5+

Installation (RobotPy on a RoboRIO)
-----------------------------------

Use robotpy-installer to install the precompiled package.

::

    robotpy-installer download-opkg python36-robotpy-pathfinder
    robotpy-installer install-opkg python36-robotpy-pathfinder

Installation (other)
--------------------

Note that this requires a C++11 compiler to be present on your system, as I'm
not currently publishing wheels of this library.

::

    pip install robotpy-pathfinder

Usage
-----

See the examples.
