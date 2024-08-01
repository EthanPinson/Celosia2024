from commands2 import SequentialCommandGroup
from wpimath.controller import PIDController
from custom.ramsetecommand import RamseteCommand
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.sewsubsystem import SewSubsystem
from constants import AutoConstants as Ac

class RamTest(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, sew: SewSubsystem):
        exampleTrajectory = sew.readJson("example")
        
        super().__init__(RamseteCommand(
            exampleTrajectory, drive.getPose2d, drive.ramsete, drive.feedforward,
            Ac.kDriveKinematics, drive.getCurrentSillySpeeds,
            PIDController(Ac.kPDriveVel, 0, 0),
            PIDController(Ac.kPDriveVel, 0, 0),
            drive.setVoltages, drive),
            drive.stopIt())

    def isFinished(self): return False