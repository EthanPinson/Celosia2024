from commands2 import SequentialCommandGroup
from wpimath.trajectory import TrajectoryGenerator
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.controller import PIDController
from custom.ramsetecommand import RamseteCommand
from subsystems.drivesubsystem import DriveSubsystem
from constants import AutoConstants as Ac

class RamTest(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem):
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            start=Pose2d(0, 0, Rotation2d(0)),
            interiorWaypoints=[Translation2d(1, 0), Translation2d(0, 1), Translation2d(-1, -1)],
            end=Pose2d(0, 0, Rotation2d(0)),
            config=drive.trajectoryConfig)
        
        super().__init__(RamseteCommand(
            exampleTrajectory, drive.getPose2d, drive.ramsete, drive.feedforward,
            Ac.kDriveKinematics, drive.getCurrentSillySpeeds,
            PIDController(Ac.kPDriveVel, 0, 0),
            PIDController(Ac.kPDriveVel, 0, 0),
            drive.setVoltages, drive),
            drive.setVoltagesC(0, 0))

    def isFinished(self) -> bool:
        return False