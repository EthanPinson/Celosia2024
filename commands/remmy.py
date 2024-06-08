from commands2 import Command
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
import commands2

class Remmy(Command):
    trajectoryConfig: TrajectoryConfig = None

    def __init__(self, drivesubsystem: DriveSubsystem):
        self.legs: DriveSubsystem = drivesubsystem()
        self.trajectoryConfig = self.legs.trajectoryConfig
        return

    def isFinished(self) -> bool:
        return False
    
    def doSomething(self):
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            start=Pose2d(0, 0, Rotation2d(0)),
            interiorWaypoints=[Translation2d(1, 1), Translation2d(2, -1)],
            end=Pose2d(3, 0, Rotation2d(0)),
            config=self.trajectoryConfig
        )