
import typing

from ._pathfinder import (
    Segment,
    
    pathfinder_modify_swerve,
    pathfinder_modify_tank,
)

__all__ = [
    'SwerveModifier',
    'TankModifier',
]


class SwerveModifier:
    """
        The Swerve Modifier will take in a Source Trajectory and spit out 4 trajectories, 1 for each wheel on the drive.
        This is commonly used in robotics for robots with 4 individual wheels in a 'swerve' configuration, where each wheel
        can rotate to a specified heading while still being powered.
        
        The Source Trajectory is measured from the centre of the drive base. The modification will not modify the central
        trajectory
    """
    
    def __init__(self, source: typing.List[Segment]) -> None:
        """Create an instance of the modifier
        :param source: The source (center) trajectory
        """
        self.source = source
        self.fl = None
        self.fr = None
        self.bl = None
        self.br = None
    
    def modify(self, wheelbase_width: float, wheelbase_depth: float) -> 'SwerveModifier':
        """Generate the Trajectory Modification
        
        :param wheelbase_width: The width (in meters) between the individual left-right sides of the drivebase
        :param wheelbase_depth: The width (in meters) between the individual front-back sides of the drivebase
        :returns:                  self
        """
        self.fl, self.fr, self.bl, self.br = pathfinder_modify_swerve(self.source, wheelbase_width, wheelbase_depth)
        return self
    
    def getSourceTrajectory(self) -> typing.List[Segment]:
        """Get the initial source trajectory"""
        return self.source
    
    def getFrontLeftTrajectory(self) -> typing.List[Segment]:
        """Get the trajectory for the front-left wheel of the drive base"""
        return self.fl
    
    def getFrontRightTrajectory(self) -> typing.List[Segment]:
        """Get the trajectory for the front-right wheel of the drive base"""
        return self.fr
    
    def getBackLeftTrajectory(self) -> typing.List[Segment]:
        """Get the trajectory for the back-left wheel of the drive base"""
        return self.bl
    
    def getBackRightTrajectory(self) -> typing.List[Segment]:
        """Get the trajectory for the back-right wheel of the drive base"""
        return self.br


class TankModifier:
    """
        The Tank Modifier will take in a Source Trajectory and a Wheelbase Width and spit out a Trajectory for each
        side of the wheelbase. This is commonly used in robotics for robots which have a drive system similar
        to a 'tank', where individual parallel sides are driven independently
        
        The Source Trajectory is measured from the centre of the drive base. The modification will not modify the central
        trajectory
    """
    
    def __init__(self, source: typing.List[Segment]) -> None:
        """Create an instance of the modifier
        :param source: The source (center) trajectory
        """
        self.source = source
        self.left = None
        self.right = None
    
    def modify(self, wheelbase_width: float) -> 'TankModifier':
        """Generate the Trajectory Modification
        
        :param wheelbase_width: The width (in meters) between the individual sides of the drivebase
        :returns:                  self
        """
        self.left, self.right = pathfinder_modify_tank(self.source, wheelbase_width)
        return self
    
    def getSourceTrajectory(self) -> typing.List[Segment]:
        """Get the initial source trajectory"""
        return self.source
    
    def getLeftTrajectory(self) -> typing.List[Segment]:
        """Get the trajectory for the left side of the drive base"""
        return self.left
    
    def getRightTrajectory(self) -> typing.List[Segment]:
        """Get the trajectory for the right side of the drive base"""
        return self.right
