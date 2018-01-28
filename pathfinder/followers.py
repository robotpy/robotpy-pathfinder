
import math
import typing

from ._pathfinder import (
    DistanceFollower as _DistanceFollower,
    EncoderFollower as _EncoderFollower,
    
    EncoderConfig,
    FollowerConfig,
    Segment,
    
    pathfinder_follow_distance2,
    pathfinder_follow_encoder2,
)

__all__ = [
    'DistanceFollower',
    'EncoderFollower',
]


class DistanceFollower(_DistanceFollower):
    """
        The DistanceFollower is an object designed to follow a trajectory based on distance covered input. This class can be used
        for Tank or Swerve drive implementations.
    """
    
    def __init__(self, trajectory: typing.List[Segment]):
        self.trajectory = trajectory
        self.cfg = FollowerConfig()
    
    def setTrajectory(self, trajectory: typing.List[Segment]) -> None:
        """Set a new trajectory to follow, and reset the cumulative errors and segment counts"""
        self.trajectory = trajectory
        self.reset()
    
    def configurePIDVA(self, kp: float, ki: float, kd: float, kv: float, ka: float) -> None:
        """Configure the PID/VA Variables for the Follower
        
        :param kp: The proportional term. This is usually quite high (0.8 - 1.0 are common values)
        :param ki: The integral term. Currently unused.
        :param kd: The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
        :param kv: The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
                   This converts m/s given by the algorithm to a scale of -1..1 to be used by your
                   motor controllers
        :param ka: The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
        """
        self.cfg.kp = kp
        self.cfg.ki = ki
        self.cfg.kd = kd
        self.cfg.kv = kv
        self.cfg.ka = ka
    
    def reset(self) -> None:
        """Reset the follower to start again. Encoders must be reconfigured."""
        self.last_error = 0
        self.segment = 0
    
    def calculate(self, distance_covered: float) -> float:
        """Calculate the desired output for the motors, based on the distance the robot has covered.
        This does not account for heading of the robot. To account for heading, add some extra terms in your control
        loop for realignment based on gyroscope input and the desired heading given by this object.
        
        :param distance_covered: The distance covered in meters
        :returns:                  The desired output for your motor controller
        """
        tlen = len(self.trajectory)
        if self.segment >= tlen:
            self.finished = 1
            self.output = 0
            self.heading = self.trajectory[-1].heading
            return 0.0
        else:
            return pathfinder_follow_distance2(self.cfg, self, self.trajectory[self.segment], tlen, distance_covered)
    
    def getHeading(self) -> float:
        """:returns: the desired heading of the current point in the trajectory"""
        return self.heading
    
    def getSegment(self) -> Segment:
        """:returns: the current segment being operated on"""
        return self.trajectory[self.segment]
    
    def isFinished(self) -> bool:
        """:returns: whether we have finished tracking this trajectory or not."""
        return self.segment >= len(self.trajectory)


class EncoderFollower(_EncoderFollower):
    """
        The EncoderFollower is an object designed to follow a trajectory based on encoder input. This class can be used
        for Tank or Swerve drive implementations.
    """
    
    def __init__(self, trajectory: typing.List[Segment]):
        self.trajectory = trajectory
        self.cfg = EncoderConfig()
    
    def setTrajectory(self, trajectory: typing.List[Segment]) -> None:
        """Set a new trajectory to follow, and reset the cumulative errors and segment counts"""
        self.trajectory = trajectory
        self.reset()
    
    def configurePIDVA(self, kp: float, ki: float, kd: float, kv: float, ka: float) -> None:
        """Configure the PID/VA Variables for the Follower
        
        :param kp: The proportional term. This is usually quite high (0.8 - 1.0 are common values)
        :param ki: The integral term. Currently unused.
        :param kd: The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
        :param kv: The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
                   This converts m/s given by the algorithm to a scale of -1..1 to be used by your
                   motor controllers
        :param ka: The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
        """
        self.cfg.kp = kp
        self.cfg.ki = ki
        self.cfg.kd = kd
        self.cfg.kv = kv
        self.cfg.ka = ka
    
    def configureEncoder(self, initial_position: int, ticks_per_revolution: int, wheel_diameter: float) -> None:
        """Configure the Encoders being used in the follower.
        
        :param initial_position: The initial 'offset' of your encoder. This should be set to the encoder value just
                                 before you start to track
        :param ticks_per_revolution: How many ticks per revolution the encoder has
        :param wheel_diameter: The diameter of your wheels (or pulleys for track systems) in meters
        """
        self.cfg.initial_position = initial_position
        self.cfg.ticks_per_revolution = ticks_per_revolution
        self.cfg.wheel_circumference = math.pi * wheel_diameter
    
    def reset(self) -> None:
        """Reset the follower to start again. Encoders must be reconfigured."""
        self.last_error = 0
        self.segment = 0
    
    def calculate(self, encoder_tick: int) -> float:
        """Calculate the desired output for the motors, based on the amount of ticks the encoder has gone through.
        This does not account for heading of the robot. To account for heading, add some extra terms in your control
        loop for realignment based on gyroscope input and the desired heading given by this object.
        
        :param encoder_tick: The amount of ticks the encoder has currently measured.
        :returns:             The desired output for your motor controller
        """
        tlen = len(self.trajectory)
        if self.segment >= tlen:
            self.finished = 1
            self.output = 0
            self.heading = self.trajectory[-1].heading
            return 0.0
        else:
            return pathfinder_follow_encoder2(self.cfg, self, self.trajectory[self.segment], tlen, encoder_tick)

    def getHeading(self) -> float:
        """:returns: the desired heading of the current point in the trajectory"""
        return self.heading
    
    def getSegment(self) -> Segment:
        """:returns: the current segment being operated on"""
        return self.trajectory[self.segment]
    
    def isFinished(self) -> bool:
        """:returns: whether we have finished tracking this trajectory or not."""
        return self.segment >= len(self.trajectory)
