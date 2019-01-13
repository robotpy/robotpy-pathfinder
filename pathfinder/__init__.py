from ._pathfinder import (
    __version__,
    Segment,
    TrajectoryInfo,
    Waypoint,
    pathfinder_generate as generate,
    pathfinder_deserialize as deserialize,
    pathfinder_serialize as serialize,
    pathfinder_serialize_csv as serialize_csv,
    pf_fit_hermite_pre as FIT_HERMITE_PRE,
    pf_fit_hermite_cubic as FIT_HERMITE_CUBIC,
    pf_fit_hermite_quintic as FIT_HERMITE_QUINTIC,
    SAMPLES_FAST,
    SAMPLES_LOW,
    SAMPLES_HIGH,
)

from . import followers, modifiers

# compability with Pathfinder-Java
import math

r2d = math.degrees
d2r = math.radians


def boundHalfDegrees(degrees):
    """Bound an angle (in degrees) to -180 to 180 degrees."""
    degrees = math.fmod(degrees, 360.0)
    if degrees >= 180.0:
        degrees = degrees - 360.0
    elif degrees <= -180.0:
        degrees = degrees + 360.0
    return degrees


def deserialize_csv(fname):
    """Read a Trajectory from a CSV File"""
    import csv

    with open(fname, "r") as fp:
        csviter = csv.reader(fp)
        next(csviter)
        return [Segment(*map(float, row)) for row in csviter]
