from commands2 import Command, SequentialCommandGroup, cmd
from RamseteCommand import RamseteCommand
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
import commands2

from constants import AutoConstants
from wpimath.controller import PIDController

class Remmy(SequentialCommandGroup):
    trajectoryConfig: TrajectoryConfig = None

    def __init__(self, drivesubsystem: DriveSubsystem):
        self.legs: DriveSubsystem = drivesubsystem
        self.trajectoryConfig = self.legs.trajectoryConfig

        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            start=Pose2d(0, 0, Rotation2d(0)),
            interiorWaypoints=[Translation2d(1, 1), Translation2d(2, -1)],
            end=Pose2d(3, 0, Rotation2d(0)),
            config=self.trajectoryConfig
        )

        ramsete = RamseteCommand(
            exampleTrajectory, self.legs.getPose2d, self.legs.ramsete, self.legs.feedforward,
            AutoConstants.kDriveKinematics, self.legs.getCurrentSpeeds,
            PIDController(AutoConstants.kPDriveVel, 0, 0),
            PIDController(AutoConstants.kPDriveVel, 0, 0),
            10, self.legs)
        
        super().__init__(ramsete)
        
        self.addCommands(cmd.runOnce(lambda: self.legs.arcadeDrive(0, 0)))
        return

    def isFinished(self) -> bool:
        return False