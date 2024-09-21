from commands2 import SequentialCommandGroup
from wpimath.controller import PIDController
from custom.ramsetecommand import RamseteCommand
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.sewsubsystem import SewSubsystem
from constants import AutoConstants as Ac
from wpimath.geometry import Rotation2d

class RamTest(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, sew: SewSubsystem):
        exampleTrajectory = sew.readJson("blueidk")
        drive.odometry.resetPosition(Rotation2d(), 0, 0, exampleTrajectory.initialPose())
        drive.resetEncoders()
        
        super().__init__(RamseteCommand(
            exampleTrajectory, drive.getPose2d, drive.ramsete, drive.feedforward,
            Ac.kDriveKinematics, drive.getCurrentSillySpeeds,
            PIDController(Ac.kPDriveVel, 0, 0),
            PIDController(Ac.kPDriveVel, 0, 0),
            drive.setVoltages, drive),
            drive.stopIt())

    def isFinished(self): return False